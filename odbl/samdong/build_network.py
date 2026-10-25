"""동네 팩의 도로망을 OSM 스냅샷에서 만든다 → network.yaml (생성물, ODbL).

    python3 -I tools/build_network.py cities/samdong
    python3 -I tools/build_network.py cities/seoul --synthetic grid=3x3 block=400 lanes=2
    python3 -I tools/build_network.py cities/tokyo --synthetic grid=3x3 block=300 lanes=1 --left

입력: <pack>/route.yaml (network_bbox, 신호 실측), <pack>/pack.yaml (lanes_seed), <pack>/osm/roads.overpass.json.
출력: <pack>/network.yaml — JSON 문법(YAML의 부분집합), sort_keys. 같은 입력 → 바이트 동일. 손으로 고치지 않는다.
설계: docs/reviews/2026-10-24-network.md. 표준 라이브러리만.

두 시계 규칙의 2D판: 호(교차로 사이 한 방향 도로) **길이**만 /6. 교차로 상자·차로 폭·회전 경로는 실제 크기.
각도는 여기서 다 소화한다 — sim엔 삼각함수가 없다 (회전 경로는 점 9개와 kappa, 호는 곡률 목록).
"""
import hashlib
import json
import math
import re
import sys

TIME_SCALE = 6.0           # SimConfig.timeScale
CURVE_MAX = 0.012          # SimConfig.curveMax (리본, 1/m) — sim의 밀림용 곡률만 자른다. 그림용 방위는 안 자른다
CURVE_SEG_RIBBON_M = 10.0  # 호 곡률 구간 (리본)
LANE_W = 3.0               # 차로 폭 (실제)
SIDEWALK_W = 2.0
MERGE_REAL_M = 40.0        # 이보다 가까운 교차 노드는 하나로 (분리 도로의 두 교차점, 램프 묶음)
MIN_ARC_RIBBON_M = 12.0    # 병합 뒤 상자 사이 리본 길이 하한 (50 km/h에서 설 수 있는 거리). 짧으면 더 병합한다
ARM_BEARING_REAL_M = 30.0  # 팔 방위: 노드에서 이만큼 간 점
ARM_GROUP_DEG = 35.0       # 같은 쪽으로 나가는 일방 호 둘(분리 도로)은 한 팔
MIN_TURN_PATH_M = 3.0      # 회전 경로가 이보다 짧으면 (반경 ~1 m) 탈 수 없다 — 뺀다. 보통 우회전은 5 m 안팎
CROSSWALK_GAP_REAL_M = 500.0   # 중간 횡단보도 간격 (v2: 330 → 500, 실제론 보행자가 누를 때만 빨강이라 차 쪽 체감이 더 길다)
CROSSWALK_SIGNAL = (100, 85)    # 주기 100초, 차량 초록 85초 (보행 버튼식의 평균 체감)
R = 6371008.8

ALLOWED = {"primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link",
           "unclassified", "residential", "living_street"}
DRIVE = {"primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified"}
RANK = {"primary": 5, "secondary": 4, "tertiary": 3, "unclassified": 2, "residential": 1, "living_street": 0}
LANES_BY_CLASS = {"primary": (2, 3), "secondary": (2, 2), "tertiary": (1, 2), "unclassified": (1, 1)}

# 신호 추정 (실제 초). build_route.py default_signal에서 옮김. 작성자 결정: 실측 없이 한국 운영 관행.
# 운영(작성자 2026-10-24 "직진, 직진·좌회전 동시, 직진 정지-좌회전"): 축마다 노드 id로 결정적으로 섞는다.
LEFT_PHASE_SHARE = 0.2
STRAIGHT_SHARE_SL = 0.65
MIN_PHASE_REAL = 30
CROSSWALK_BOX_HALF = 4.5   # 중간 횡단보도 노드 상자 반깊이 (m): 건널목 반폭 1.6 + 정지선 앞 여유
BOUNDARY_STUB_MIN_RIBBON_M = 12.0  # 경계로 가는 호가 리본으로 이보다 짧으면 지운다 (지도를 자른 자리 — 길이 아니다)
RIRO_MAX_SIGNAL_ARMS = 4    # 팔 5개 이상 교차로: 신호 팔은 이만큼, 나머지 부차 팔은 우회전 전용 (주기 ≤ 40 리본초)
STRAIGHT_BEND_DEG = 10.0    # 들어오는·나가는 팔이 이보다 어긋난 직진은 9점 곡선 (6갈래의 꺾인 직진)
CONFLICT_NEAR_M = 2.6        # 두 경로 중심선이 이보다 가까이 지나면 충돌 (버스 2.5 m·차 1.8 m 폭, 거의 나란한 팔의 입구)
PAVE_STEP_M = 0.1           # 포장 끝 재기 (경로 법선으로 걷는 간격)
PAVE_MAX_M = 6.0
UTURN_FWD_MAX = 2.75       # 유턴 반원이 상자 안으로 부푸는 깊이 상한 (m). 상자 반깊이를 넘지 않는다 (docs/reviews/2026-10-24-uturn.md)


def default_signal(a_cls, b_cls, link):
    a, b = RANK.get(a_cls, 0), RANK.get(b_cls, 0)
    if link:
        return (120, 60)
    if a >= 5:
        return (150, 70) if b >= 3 else (130, 85)
    if a >= 3:
        return (120, 55) if b >= 3 else (100, 60)
    return (120, 40)


def base_cls(hw):
    return hw[:-5] if hw.endswith("_link") else hw


def dist(a, b):
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    return 2 * R * math.asin(math.sqrt(math.sin((p2 - p1) / 2) ** 2 +
                                       math.cos(p1) * math.cos(p2) * math.sin(math.radians(b[1] - a[1]) / 2) ** 2))


def bearing(a, b):
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    return math.atan2(math.sin(dl) * math.cos(p2), math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def h32(*parts):
    return int.from_bytes(hashlib.sha256(":".join(str(p) for p in parts).encode()).digest()[:4], "big")


def poly_len(pts):
    return sum(dist(a, b) for a, b in zip(pts, pts[1:]))


def point_along(pts, d):
    """폴리라인 위 시작에서 실거리 d인 점."""
    for a, b in zip(pts, pts[1:]):
        L = dist(a, b)
        if d <= L or b is pts[-1]:
            f = 0 if L == 0 else min(1.0, d / L)
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
        d -= L
    return pts[-1]


def split_poly(pts, cuts):
    """실거리 cuts(오름차순)에서 폴리라인을 자른다 → 조각 목록."""
    out, cur, acc, ci = [], [pts[0]], 0.0, 0
    for a, b in zip(pts, pts[1:]):
        L = dist(a, b)
        while ci < len(cuts) and acc + L >= cuts[ci]:
            f = 0 if L == 0 else (cuts[ci] - acc) / L
            p = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
            cur.append(p)
            out.append(cur)
            cur = [p]
            ci += 1
        cur.append(b)
        acc += L
    out.append(cur)
    return out


def read_kv(text, key):
    m = re.search(rf"^{key}:\s*(.+?)\s*(#.*)?$", text, re.M)
    return m.group(1) if m else None


def read_list(text, key):
    v = read_kv(text, key)
    return None if v is None else [float(x) for x in v.strip("[]").split(",")]


def read_signals(text):
    """route.yaml signals: 아래 `- {osm_node: N, lunch_cycle: .., ...}` 줄들 (실측, 선택)."""
    out = []
    m = re.search(r"^signals:\s*\n((?:\s+-.*\n?)*)", text, re.M)
    if not m:
        return out
    for line in m.group(1).splitlines():
        body = line.strip().lstrip("-").strip().strip("{}")
        if not body or body.startswith("#"):
            continue
        d = {}
        for kv in body.split(","):
            k, _, v = kv.partition(":")
            v = v.strip().strip('"')
            try:
                v = int(v)
            except ValueError:
                pass
            d[k.strip()] = v
        out.append(d)
    return out


# ───────────────────────── 그래프 (OSM → 교차로·사슬) ─────────────────────────

def load_osm(pack_dir, bbox):
    snap = json.load(open(f"{pack_dir}/osm/roads.overpass.json", encoding="utf-8"))
    nodes = {e["id"]: (e["lat"], e["lon"]) for e in snap["elements"] if e["type"] == "node"}
    lon0, lat0, lon1, lat1 = bbox

    def inside(n):
        la, lo = nodes[n]
        return lat0 <= la <= lat1 and lon0 <= lo <= lon1

    seg = {}       # (a, b) 무방향 키 a<b → {fwd, bwd, cls, way, lanes tags}
    stubs = {}     # 주행 노드 → [골목 방위 노드]
    for e in sorted((e for e in snap["elements"] if e["type"] == "way"), key=lambda e: e["id"]):
        t = e.get("tags", {})
        hw = t.get("highway", "")
        if hw not in ALLOWED or t.get("motorroad") == "yes" or t.get("motorcycle") == "no":
            continue
        if t.get("access") in ("no", "private") or t.get("area") == "yes":
            continue
        ns = [n for n in e["nodes"] if n in nodes]
        if hw not in DRIVE:
            for a, b in zip(ns, ns[1:]):
                stubs.setdefault(a, []).append(b)
                stubs.setdefault(b, []).append(a)
            continue
        ow = t.get("oneway", "no")
        if t.get("junction") == "roundabout" and ow == "no":
            ow = "yes"
        fwd = ow != "-1"
        bwd = ow not in ("yes", "1", "true")
        for a, b in zip(ns, ns[1:]):
            if not (inside(a) or inside(b)):
                continue
            k = (a, b) if a < b else (b, a)
            f, bk = (fwd, bwd) if a < b else (bwd, fwd)
            seg[k] = {"fwd": f, "bwd": bk, "cls": hw, "way": e["id"], "tags": t, "rev": a > b}
    return snap, nodes, inside, seg, stubs


def chains(nodes, inside, seg):
    adj = {}
    for (a, b) in seg:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    for n in adj:
        adj[n].sort()
    term = {n for n in adj if len(adj[n]) != 2 or not inside(n)}
    seen = set()
    out = []
    for s in sorted(term):
        for nb in adj[s]:
            k = (s, nb) if s < nb else (nb, s)
            if k in seen:
                continue
            path = [s, nb]
            seen.add(k)
            while path[-1] not in term:
                cur, prev = path[-1], path[-2]
                nxt = adj[cur][0] if adj[cur][0] != prev else adj[cur][1]
                kk = (cur, nxt) if cur < nxt else (nxt, cur)
                seen.add(kk)
                path.append(nxt)
                if nxt == s:
                    break
            if path[-1] == s:
                continue  # 고리 (교차 없음) — 버린다
            fwd = all(seg[(a, b) if a < b else (b, a)]["fwd" if a < b else "bwd"] for a, b in zip(path, path[1:]))
            bwd = all(seg[(a, b) if a < b else (b, a)]["bwd" if a < b else "fwd"] for a, b in zip(path, path[1:]))
            if not (fwd or bwd):
                continue
            segs = [seg[(a, b) if a < b else (b, a)] for a, b in zip(path, path[1:])]
            cls = max((s_["cls"] for s_ in segs), key=lambda c: (RANK.get(base_cls(c), 0), not c.endswith("_link")))
            out.append({"nodes": path, "fwd": fwd, "bwd": bwd, "cls": cls, "link": cls.endswith("_link"),
                        "ways": sorted({s_["way"] for s_ in segs}), "tags": segs[len(segs) // 2]["tags"],
                        "rev_tags": segs[len(segs) // 2]["rev"]})
    return term, out


# ───────────────────────── 메인 ─────────────────────────

def build(pack_dir):
    route_text = open(f"{pack_dir}/route.yaml", encoding="utf-8").read()
    pack_text = open(f"{pack_dir}/pack.yaml", encoding="utf-8").read()
    bbox = read_list(route_text, "network_bbox")
    if bbox is None:
        raise SystemExit("route.yaml에 network_bbox: [lon0, lat0, lon1, lat1]가 없다")
    lanes_seed = int(read_kv(pack_text, "lanes_seed") or 1)
    side = -1 if (read_kv(pack_text, "traffic_side") or "right") == "left" else 1
    snap, nodes, inside, seg, stubs = load_osm(pack_dir, bbox)
    term, ch = chains(nodes, inside, seg)

    # 사슬 → 간선. 긴 간선엔 중간 횡단보도 노드.
    pos = {}       # 노드 키 → (lat, lon)
    kind = {}      # junction | boundary | crosswalk
    for n in term:
        pos[n] = nodes[n]
        kind[n] = "junction" if inside(n) and len(set(_nb(seg, n))) >= 3 else "boundary"
    edges = []
    for ci, c in enumerate(ch):
        pts = [nodes[n] for n in c["nodes"]]
        L = poly_len(pts)
        a, b = c["nodes"][0], c["nodes"][-1]
        n_cw = int(L // CROSSWALK_GAP_REAL_M) if kind[a] == kind[b] == "junction" else 0
        if n_cw:
            pieces = split_poly(pts, [L * (m + 1) / (n_cw + 1) for m in range(n_cw)])
            keys = [a] + [f"cw{ci}_{m}" for m in range(n_cw)] + [b]
            for m in range(n_cw):
                pos[keys[m + 1]] = pieces[m][-1]
                kind[keys[m + 1]] = "crosswalk"
        else:
            pieces, keys = [pts], [a, b]
        for m, p in enumerate(pieces):
            edges.append({"a": keys[m], "b": keys[m + 1], "pts": p, "len": poly_len(p), "fwd": c["fwd"], "bwd": c["bwd"],
                          "cls": c["cls"], "link": c["link"], "way": c["ways"][0], "tags": c["tags"], "rev_tags": c["rev_tags"],
                          "osm": c["nodes"] if len(pieces) == 1 else None})

    # 차로 수 (실제 태그 > 도구가 시드로 뽑아 팩에 적는다 — sim은 기하를 발명하지 않는다)
    for e in edges:
        t = e["tags"]
        lf, lb = t.get("lanes:forward"), t.get("lanes:backward")
        if e["rev_tags"]:
            lf, lb = lb, lf
        lanes = t.get("lanes")
        lo, hi = LANES_BY_CLASS.get(base_cls(e["cls"]), (1, 1))
        seeded = lo + h32(lanes_seed, "lanes", e["way"]) % (hi - lo + 1)
        if lf and lb and lf.isdigit() and lb.isdigit():
            f, b = int(lf), int(lb)
        elif lanes and lanes.isdigit():
            n = int(lanes)
            f = b = max(1, n // 2) if (e["fwd"] and e["bwd"]) else n
        else:
            f = b = seeded
        if e["link"]:
            f = b = 1
        e["lf"] = min(3, max(1, f)) if e["fwd"] else 0
        e["lb"] = min(3, max(1, b)) if e["bwd"] else 0

    # 병합: 실거리 40 m 안의 교차 노드, 그 뒤 상자 사이 리본이 짧은 간선도 반복해서
    parent = {k: k for k in pos}

    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    def union(x, y):
        x, y = find(x), find(y)
        if x != y:
            a, b = sorted([x, y], key=str)
            parent[b] = a

    for e in edges:
        if kind[e["a"]] == kind[e["b"]] == "junction" and e["len"] < MERGE_REAL_M:
            union(e["a"], e["b"])
    for _ in range(20):
        half = box_halves(edges, kind, find)
        changed = False
        for e in edges:
            ra, rb = find(e["a"]), find(e["b"])
            if ra == rb or kind[e["a"]] == "boundary" or kind[e["b"]] == "boundary":
                continue
            if e["len"] / TIME_SCALE - half.get(ra, 0) - half.get(rb, 0) < MIN_ARC_RIBBON_M:
                if kind[e["a"]] == "crosswalk" or kind[e["b"]] == "crosswalk":
                    # 횡단보도는 교차로에 흡수시킨다 (횡단보도끼리는 합치지 않는다 — 그건 짧은 대로 둔다)
                    if kind[e["a"]] == kind[e["b"]] == "crosswalk":
                        union(e["a"], e["b"])  # 이웃한 횡단보도 둘 → 하나 (같은 사슬이라 팔은 둘 그대로)
                        changed = True
                        continue
                    cw = e["a"] if kind[e["a"]] == "crosswalk" else e["b"]
                    other = e["b"] if cw == e["a"] else e["a"]
                    union(cw, other)
                else:
                    union(e["a"], e["b"])
                changed = True
        if not changed:
            break

    # 대표 노드: 좌표 평균, 종류는 교차로 > 횡단보도 > 경계
    groups = {}
    for k in pos:
        groups.setdefault(find(k), []).append(k)
    rank_kind = {"junction": 3, "crosswalk": 2, "boundary": 1}
    rep = {}
    for r_, ms in groups.items():
        lat = sum(pos[m][0] for m in ms) / len(ms)
        lon = sum(pos[m][1] for m in ms) / len(ms)
        rep[r_] = {"lat": lat, "lon": lon, "kind": max((kind[m] for m in ms), key=lambda k: rank_kind[k]), "members": sorted(ms, key=str)}
    live = [e for e in edges if find(e["a"]) != find(e["b"])]
    # 지도 경계 짧은 호 삭제 (crash-rules 리뷰 C0): 경계 노드로 가는 간선이 리본 12 m 미만이면 길이 아니라 자른 자리다.
    # 교차로 쪽엔 막힌 골목처럼 그림(stub 방위)만 남긴다.
    half0 = box_halves(live, kind, find)

    def hb_of(r_):
        k_ = rep[r_]["kind"]
        return half0.get(r_, 7.0) if k_ == "junction" else (CROSSWALK_BOX_HALF if k_ == "crosswalk" else 3.0)

    extra_stubs = {}
    kept = []
    for e in live:
        ra, rb = find(e["a"]), find(e["b"])
        bnd = [r_ for r_ in (ra, rb) if rep[r_]["kind"] == "boundary"]
        if bnd and e["len"] / TIME_SCALE - hb_of(ra) - hb_of(rb) < BOUNDARY_STUB_MIN_RIBBON_M:
            inner = rb if ra in bnd else ra
            if rep[inner]["kind"] != "boundary":
                far = e["pts"][-1] if inner == ra else e["pts"][0]
                o = (rep[inner]["lat"], rep[inner]["lon"])
                extra_stubs.setdefault(inner, set()).add(round(math.degrees(bearing(o, far)) % 360, 1))
            stats_removed = extra_stubs.setdefault("_n", set())
            stats_removed.add(id(e))
            continue
        kept.append(e)
    removed_stubs = len(extra_stubs.pop("_n", set()))
    live = kept
    # 간선에 이어지지 않은 노드는 버린다
    used = {find(e["a"]) for e in live} | {find(e["b"]) for e in live}
    keys = sorted(used, key=lambda r_: (rep[r_]["lat"], rep[r_]["lon"]))
    nid = {r_: i for i, r_ in enumerate(keys)}
    half = box_halves(live, kind, find)

    # 호 (방향 간선)
    arcs = []
    for e in sorted(live, key=lambda e: (nid[find(e["a"])], nid[find(e["b"])], e["way"], e["len"])):
        a, b = nid[find(e["a"])], nid[find(e["b"])]
        for fwd in (True, False):
            if (fwd and not e["fwd"]) or (not fwd and not e["bwd"]):
                continue
            pts = e["pts"] if fwd else list(reversed(e["pts"]))
            arcs.append({"id": len(arcs), "from": a if fwd else b, "to": b if fwd else a, "pts": pts, "len_real": e["len"],
                         "lanes": e["lf"] if fwd else e["lb"], "opp": e["lb"] if fwd else e["lf"], "cls": e["cls"],
                         "link": e["link"], "edge": id(e), "way": e["way"], "osm": e["osm"] if fwd else (list(reversed(e["osm"])) if e["osm"] else None)})
    for ar in arcs:
        twin = [o for o in arcs if o["edge"] == ar["edge"] and o is not ar]
        ar["twin"] = twin[0]["id"] if twin else None

    # 팔: 노드마다 들어오는/나가는 호를 방위로 묶는다
    node_out = []
    for r_ in keys:
        i = nid[r_]
        ends = []
        for ar in arcs:
            if ar["to"] == i:
                p = ar["pts"][::-1]
                ends.append(("in", ar, bearing(p[0], point_along(p, min(ARM_BEARING_REAL_M, ar["len_real"] / 2)))))
            if ar["from"] == i:
                p = ar["pts"]
                ends.append(("out", ar, bearing(p[0], point_along(p, min(ARM_BEARING_REAL_M, ar["len_real"] / 2)))))
        ends.sort(key=lambda t: (t[2], t[1]["id"]))
        arms = []
        for io, ar, br in ends:
            home = None
            for arm in arms:
                if abs(wrap(arm["b"] - br)) < math.radians(ARM_GROUP_DEG) and arm[io] is None:
                    home = arm
                    break
            if home is None:
                home = {"in": None, "out": None, "b": br, "bs": []}
                arms.append(home)
            home[io] = ar
            home["bs"].append(br)
        for arm in arms:
            # 방위 평균 (작은 각도 차라 단순 평균으로 충분)
            b0 = arm["bs"][0]
            arm["b"] = wrap(b0 + sum(wrap(x - b0) for x in arm["bs"]) / len(arm["bs"]))
        arms.sort(key=lambda a_: (a_["b"] % (2 * math.pi)))
        node_out.append({"key": r_, "arms": arms})

    # 상자·이동 경로·신호
    surveyed = {int(s_["osm_node"]): s_ for s_ in read_signals(route_text) if "osm_node" in s_}
    uturn_survey = read_uturns(route_text)
    arcs_by_id = {ar["id"]: ar for ar in arcs}
    out_nodes = []
    stats = {"junction": 0, "crosswalk": 0, "boundary": 0, "signal": 0, "moves": 0, "warn": [], "stubs_removed": removed_stubs, "riro": 0}
    for nd in node_out:
        r_, arms = nd["key"], nd["arms"]
        i = nid[r_]
        info = rep[r_]
        k = info["kind"]
        if k == "junction" and len(arms) < 3:
            k = "joint" if len(arms) == 2 else "boundary"
        stats[k if k in stats else "junction"] = stats.get(k if k in stats else "junction", 0) + 1
        # 횡단보도 노드: 건널목(±1.6 m) 앞 3 m에 정지선 — 그 사이에서 유턴한다 (보행신호시 유턴)
        hb = half.get(r_, 7.0) if k == "junction" else (CROSSWALK_BOX_HALF if k == "crosswalk" else (0.0 if k == "joint" else 3.0))
        if k == "joint" and abs(abs(math.degrees(wrap(arms[0]["b"] - arms[1]["b"]))) - 180) > 30:
            hb = half.get(r_, 7.0)  # 꺾인 이음 (경계 짧은 호를 지운 모퉁이): 상자 없이 돌면 반경 1 m짜리 회전이 된다
        moves = []
        for ai, arm in enumerate(arms):
            ain = arm["in"]
            if ain is None:
                continue
            h_in = wrap(arm["b"] + math.pi)
            by_kind = {}
            uturn_cands = []
            for aj, arm2 in enumerate(arms):
                if aj == ai or arm2["out"] is None:
                    continue
                d = wrap(arm2["b"] - h_in)
                deg = math.degrees(d)
                if abs(deg) <= 30:
                    mk = "straight"
                elif abs(deg) <= 135:  # 그 너머는 유턴에 가깝다 (경로가 들어오는 차로를 가로지른다) — v1 없음
                    mk = "right" if deg > 0 else "left"
                else:
                    uturn_cands.append((abs(deg), aj, d))
                    continue
                if side < 0 and mk != "straight":
                    pass  # 좌측통행이어도 방향 이름은 같다 (오른쪽으로 꺾으면 right)
                score = abs(deg) if mk == "straight" else abs(abs(deg) - 90)
                if mk not in by_kind or score < by_kind[mk][0]:
                    if mk in by_kind:
                        stats["warn"].append(f"노드 {i}: 팔 {ai}에서 {mk} 후보 둘")
                    by_kind[mk] = (score, aj, d)
            if not by_kind and uturn_cands:
                # 나갈 길이 유턴성 회전뿐인 팔 (분리 도로가 갈라지는 곳): 그거라도 남긴다. 가로지르는 회전으로 친다
                _, aj, d = min(uturn_cands)
                by_kind["left" if side > 0 else "right"] = (0, aj, d)
                stats["warn"].append(f"노드 {i}: 팔 {ai} 출구가 유턴성 회전뿐 — 남긴다")
            if k in ("crosswalk", "joint"):
                by_kind = {kk: v for kk, v in by_kind.items() if kk == "straight"} or by_kind
            for mk in ("left", "straight", "right"):
                if mk not in by_kind:
                    continue
                _, aj, d = by_kind[mk]
                aout = arms[aj]["out"]
                L_in, L_out = ain["lanes"], aout["lanes"]
                # 왼쪽(중앙선 쪽) 0 … 연석 L-1. 좌측통행이면 '중앙선 쪽'이 오른쪽이다 — 같은 번호 체계.
                near_center = (mk == "left") if side > 0 else (mk == "right")
                near_curb = (mk == "right") if side > 0 else (mk == "left")
                lf = 0 if near_center else (L_in - 1 if near_curb else 0)
                lt = 0 if near_center else (L_out - 1 if near_curb else min(lf, L_out - 1))
                path, kappa = move_path(arm["b"], arms[aj]["b"], hb, lf, lt, side, mk)
                plen = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
                if mk != "straight" and plen < MIN_TURN_PATH_M and len(by_kind) > 1:
                    # 너무 꺾인 회전은 뺀다 — 단, 그 팔의 유일한 출구면 남긴다 (나갈 길 없는 호는 없다)
                    stats["warn"].append(f"노드 {i}: 팔 {ai}→{aj} {mk} 경로 {plen:.1f} m — 너무 꺾여 뺀다")
                    continue
                moves.append({"from_arc": ain["id"], "to_arc": aout["id"], "kind": mk, "from_arm": ai, "to_arm": aj,
                              "lane_from": lf, "lane_to": lt, "path": path, "kappa": kappa})
            # 유턴: 들어오는 호에 쌍둥이(같은 팔로 나가는 반대 방향 호)가 있으면 (교차로·횡단보도·경계). 합법 여부(cond)는 신호 뒤에
            if k != "joint" and ain["twin"] is not None and arm["out"] is not None and arm["out"]["id"] == ain["twin"]:
                # 횡단보도 노드는 건널목(중심 ±1.6 m) 앞에서 돈다 (보행신호시 유턴)
                path, kappa, lt = uturn_path(arm["b"], hb, ain["opp"], side, cap=hb - 3.0 if k == "crosswalk" else hb)
                moves.append({"from_arc": ain["id"], "to_arc": ain["twin"], "kind": "uturn", "from_arm": ai, "to_arm": ai,
                              "lane_from": 0, "lane_to": lt, "path": path, "kappa": kappa, "cond": "none"})
        # 팔 5개 이상: 등급·차로가 낮은 팔부터 우회전 전용 (신호 팔 ≤ 4 → 단계 ≤ 4, 주기 ≤ 40 리본초). route.yaml signals:가 이긴다
        riro = set()
        if k == "junction" and not any(m_ in surveyed for m_ in info["members"] if isinstance(m_, int)):
            sig_arms = [ai for ai, a_ in enumerate(arms) if a_["in"] is not None]
            if len(sig_arms) > RIRO_MAX_SIGNAL_ARMS:
                def weight(ai):
                    ar = arms[ai]["in"] or arms[ai]["out"]
                    lanes = (arms[ai]["in"]["lanes"] if arms[ai]["in"] else 0) + (arms[ai]["out"]["lanes"] if arms[ai]["out"] else 0)
                    return (RANK.get(base_cls(ar["cls"]), 0), lanes, -ai)
                for ai in sorted(sig_arms, key=weight)[:len(sig_arms) - RIRO_MAX_SIGNAL_ARMS]:
                    riro.add(ai)
                near = "right" if side > 0 else "left"
                moves = [m for m in moves if not ((m["from_arm"] in riro or m["to_arm"] in riro) and m["kind"] != near)]
                stats["riro"] += len(riro)
        stats["moves"] += len(moves)
        annotate_moves(moves, arms, hb)
        signal = None
        if k == "crosswalk":
            signal = {"cycle": CROSSWALK_SIGNAL[0], "crosswalk": True,
                      "axes": [{"arms": [0, 1] if len(arms) > 1 else [0], "plan": "straight", "straight": CROSSWALK_SIGNAL[1], "left": 0,
                                "unprotected_left": False}]}
        elif k == "junction":
            signal = junction_signal(i, arms, moves, surveyed, info["members"], skip=riro)
        if signal:
            stats["signal"] += 1
        for m in moves:
            if m["kind"] == "uturn":
                m["cond"] = uturn_cond(k, signal, m, arcs_by_id, uturn_survey, info["members"])
                stats["uturn_" + m["cond"]] = stats.get("uturn_" + m["cond"], 0) + 1
        out_nodes.append({
            "id": i, "lat": round(info["lat"], 7), "lon": round(info["lon"], 7), "kind": k, "box_half": round(hb, 2),
            "arms": [{"arc_in": a_["in"]["id"] if a_["in"] else None, "arc_out": a_["out"]["id"] if a_["out"] else None,
                      "bearing_deg": round(math.degrees(a_["b"]) % 360, 2),
                      "ux": round(math.sin(a_["b"]), 6), "uy": round(math.cos(a_["b"]), 6),
                      **({"riro": True} if ai in riro else {})} for ai, a_ in enumerate(arms)],
            "moves": moves,
            "signal": signal,
            "stubs": sorted({round(math.degrees(bearing(pos_of(m, pos), nodes[s_])) % 360, 1)
                             for m in info["members"] if m in stubs for s_ in stubs[m]} | extra_stubs.get(r_, set())),
        })

    # 호: 곡률(리본, sim 밀림용 클램프)과 방위(그림용, 실제), 골목 입구
    out_arcs = []
    for ar in arcs:
        pts = ar["pts"]
        Lr = ar["len_real"]
        seg_real = CURVE_SEG_RIBBON_M * TIME_SCALE
        n_seg = max(1, round(Lr / seg_real))
        heads = []
        for m in range(n_seg + 1):
            s0 = min(Lr - 0.5, max(0.0, m * Lr / n_seg - seg_real / 4))
            s1 = min(Lr, s0 + max(1.0, min(seg_real / 2, Lr / 2)))
            heads.append(bearing(point_along(pts, s0), point_along(pts, s1)))
        unwrapped = [heads[0]]
        for h in heads[1:]:
            unwrapped.append(unwrapped[-1] + wrap(h - unwrapped[-1]))
        seg_rib = Lr / n_seg / TIME_SCALE
        curves = []
        for m in range(n_seg):
            kk = (unwrapped[m + 1] - unwrapped[m]) / seg_rib
            curves.append(round(max(-CURVE_MAX, min(CURVE_MAX, kk)), 5))
        astubs = []
        if ar["osm"]:
            acc = 0.0
            for j, n in enumerate(ar["osm"]):
                if j > 0:
                    acc += dist(nodes[ar["osm"][j - 1]], nodes[n])
                if 0 < j < len(ar["osm"]) - 1 and n in stubs:
                    hdg = bearing(nodes[ar["osm"][j - 1]], nodes[ar["osm"][j + 1]])
                    for s_ in stubs[n]:
                        rel = math.degrees(wrap(bearing(nodes[n], nodes[s_]) - hdg))
                        if 30 < abs(rel) < 150:
                            astubs.append({"s_real": round(acc, 1), "side": "right" if rel > 0 else "left"})
        out_arcs.append({
            "id": ar["id"], "from": ar["from"], "to": ar["to"], "len_real": round(Lr, 1), "lanes": ar["lanes"], "opp": ar["opp"],
            "twin": ar["twin"], "class": ar["cls"], "curves": curves,
            "heads_deg": [round(math.degrees(h), 2) for h in unwrapped],
            "polyline": [[round(p[0], 7), round(p[1], 7)] for p in pts],
            "stubs": sorted(astubs, key=lambda s_: (s_["s_real"], s_["side"])),
        })
    return snap, out_nodes, out_arcs, stats, side


def pos_of(m, pos):
    return pos[m]


def _nb(seg, n):
    for (a, b) in seg:
        if a == n:
            yield b
        elif b == n:
            yield a


def box_halves(edges, kind, find):
    """상자 반깊이 (실제 m): 노드에 닿는 가장 넓은 도로의 반폭 + 인도. 7~12 m."""
    width = {}
    for e in edges:
        w = (e["lf"] + e["lb"]) * LANE_W
        for k in (e["a"], e["b"]):
            if kind[k] == "junction":
                r_ = find(k)
                width[r_] = max(width.get(r_, 0), w)
    return {r_: max(7.0, min(12.0, w / 2 + SIDEWALK_W)) for r_, w in width.items()}


def move_path(b_in_arm, b_out_arm, hb, lf, lt, side, mk):
    """상자 안 경로: 노드 국소 평면 (x 동, y 북, m). 들어오는 차로 중심에서 나가는 차로 중심까지 2차 베지어 9점."""
    h_in = wrap(b_in_arm + math.pi)
    h_out = b_out_arm

    def d(h):
        return (math.sin(h), math.cos(h))

    def right(h):
        return (math.cos(h), -math.sin(h))

    o_in, o_out = (lf + 0.5) * LANE_W * side, (lt + 0.5) * LANE_W * side
    E = (d(b_in_arm)[0] * hb + right(h_in)[0] * o_in, d(b_in_arm)[1] * hb + right(h_in)[1] * o_in)
    X = (d(h_out)[0] * hb + right(h_out)[0] * o_out, d(h_out)[1] * hb + right(h_out)[1] * o_out)
    dlt = wrap(h_out - h_in)
    if mk == "straight" and abs(math.degrees(dlt)) <= STRAIGHT_BEND_DEG:
        pts = [E, X]
    else:
        d1, d2 = d(h_in), d(h_out)
        den = d1[0] * d2[1] - d1[1] * d2[0]
        C = None
        if abs(den) > 1e-6:
            wx, wy = X[0] - E[0], X[1] - E[1]
            t = (wx * d2[1] - wy * d2[0]) / den
            u = (wx * d1[1] - wy * d1[0]) / den
            if t > 0 and u > 0:
                C = (E[0] + d1[0] * t, E[1] + d1[1] * t)
        if C is None:
            # 두 직선이 앞에서 안 만나면 (완만한 꺾임·어긋난 팔) 들어오는 방향으로 뻗은 점: 경로가 늘 진입 방향에서 부드럽게 시작한다
            half_d = math.hypot(X[0] - E[0], X[1] - E[1]) / 2
            C = (E[0] + d1[0] * half_d, E[1] + d1[1] * half_d)
        pts = []
        for m in range(9):
            t = m / 8
            pts.append(((1 - t) ** 2 * E[0] + 2 * (1 - t) * t * C[0] + t * t * X[0],
                        (1 - t) ** 2 * E[1] + 2 * (1 - t) * t * C[1] + t * t * X[1]))
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    kappa = 0.0 if len(pts) == 2 else dlt / max(L, 1.0)
    return [[round(p[0], 3), round(p[1], 3)] for p in pts], round(kappa, 5)


def _at(path, f):
    """경로 위 길이 f의 점과 단위 접선 (sim NetPath.at와 같은 정의)."""
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    i = 0
    while i < len(cum) - 2 and cum[i + 1] < f:
        i += 1
    seg = cum[i + 1] - cum[i]
    r = 0.0 if seg <= 0 else max(0.0, min(1.0, (f - cum[i]) / seg))
    a, b = path[i], path[i + 1]
    tx, ty = b[0] - a[0], b[1] - a[1]
    n = math.hypot(tx, ty) or 1.0
    return (a[0] + tx * r, a[1] + ty * r), (tx / n, ty / n), cum[-1]


def _seg_cross(p1, p2, q1, q2):
    def o(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2, d3, d4 = o(q1, q2, p1), o(q1, q2, p2), o(p1, p2, q1), o(p1, p2, q2)
    return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0)


def _dense(path, n=24):
    L = _at(path, 0)[2]
    return [_at(path, L * k / n)[0] for k in range(n + 1)]


def annotate_moves(moves, arms, hb):
    """동작마다 (crash-rules 리뷰 C0)
    - conflicts: 같은 노드에서 기하적으로 부딪히는 동작 번호 — 경로가 교차하거나 1.6 m보다 가까이 지나거나(차 반폭 둘),
      같은 호로 나간다(합류, 차로 무관). 같은 팔에서 들어오는 동작끼리는 아니다 (앞뒤로 따라간다).
    - off_lo / off_hi: 경로 9점에서 포장 끝까지 (법선, m). 포장 = 팔 직사각형(반폭 = 양방향 차로 × 1.5 m) ∪ 가장 좁은 팔 반폭의 원.
      sim의 '연석'은 이 띠 밖이다."""
    hws = []
    for a_ in arms:
        lanes = (a_["in"]["lanes"] if a_["in"] else 0) + (a_["out"]["lanes"] if a_["out"] else 0)
        if a_["in"] and not a_["out"]:
            lanes += a_["in"]["opp"]
        if a_["out"] and not a_["in"]:
            lanes += a_["out"]["opp"]
        hws.append(max(1, lanes) * LANE_W / 2)
    r0 = min(hws) if hws else 3.0

    def paved(x, y):
        if x * x + y * y <= r0 * r0:
            return True
        for a_, hw in zip(arms, hws):
            ux, uy = math.sin(a_["b"]), math.cos(a_["b"])
            along = x * ux + y * uy
            lat = abs(x * uy - y * ux)
            if -1.0 <= along <= hb + 60 and lat <= hw:
                return True
        return False

    for m in moves:
        lo, hi = [], []
        L = _at(m["path"], 0)[2]
        for k in range(9):
            (px, py), (tx, ty), _ = _at(m["path"], L * k / 8)
            nx, ny = ty, -tx  # 오른쪽 법선
            out = []
            for sgn in (1, -1):
                d = 0.0
                while d < PAVE_MAX_M and paved(px + nx * sgn * (d + PAVE_STEP_M), py + ny * sgn * (d + PAVE_STEP_M)):
                    d += PAVE_STEP_M
                out.append(round(max(0.4, d), 2))
            hi.append(out[0])
            lo.append(-out[1])
        m["off_lo"], m["off_hi"] = lo, hi
    dense = [_dense(m["path"]) for m in moves]
    for i, m in enumerate(moves):
        conf = []
        for j, o in enumerate(moves):
            if j == i or o["from_arm"] == m["from_arm"]:
                continue
            hit = False
            if o["to_arc"] == m["to_arc"]:
                hit = True  # 같은 호로 합류 (차로가 달라도: 플레이어·차는 들어온 차로 오프셋 그대로 나간다)
            else:
                pa, pb = dense[i], dense[j]
                for a1, a2 in zip(pa, pa[1:]):
                    for b1, b2 in zip(pb, pb[1:]):
                        if _seg_cross(a1, a2, b1, b2):
                            hit = True
                            break
                    if hit:
                        break
                if not hit:
                    # 나란히 가까이 (같은 출구 다른 차로는 끝 2 m 빼고)
                    cut = 3 if o["to_arc"] == m["to_arc"] else 0
                    for p in pa[:len(pa) - cut]:
                        for q in pb[:len(pb) - cut]:
                            if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 < CONFLICT_NEAR_M ** 2:
                                hit = True
                                break
                        if hit:
                            break
            if hit:
                conf.append(j)
        m["conflicts"] = conf
    return moves


def uturn_path(b_arm, hb, opp, side, cap):
    """유턴 경로: 정지선의 중앙선(x=0)에서 들어와 상자 안으로 반타원을 그리고, 쌍둥이 호의 차로 중심으로 같은 팔을 나간다.
    앞으로 부푸는 깊이 r = clamp(1.5·opp − 0.25, 1.25, 2.75), 상자 반깊이 이하. 9점."""
    h_in = wrap(b_arm + math.pi)
    lt = 0 if opp <= 1 else 1
    w = (lt + 0.5) * LANE_W          # 중앙선에서 나갈 차로 중심까지 (가로 폭)
    r = max(0.5, min(max(1.25, min(UTURN_FWD_MAX, 1.5 * opp - 0.25)), cap))
    fx, fy = math.sin(h_in), math.cos(h_in)      # 진행 방향
    rx, ry = math.cos(h_in), -math.sin(h_in)     # 오른쪽
    ex, ey = math.sin(b_arm) * hb, math.cos(b_arm) * hb  # 정지선 중앙 (팔 쪽 상자 가장자리)
    pts = []
    for m in range(9):
        t = math.pi * m / 8
        f = r * math.sin(t)
        lat = -side * w / 2 * (1 - math.cos(t))   # 중앙선 너머 (좌측통행이면 오른쪽)
        pts.append([round(ex + fx * f + rx * lat, 3), round(ey + fy * f + ry * lat, 3)])
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    kappa = -side * math.pi / max(L, 1.0)
    return pts, round(kappa, 5), lt


def read_uturns(text):
    """route.yaml `uturns:` 실측 — `- {node: <osm node id>, way: <osm way id 생략 가능>, cond: left|ped|always|none}`. 추정을 이긴다."""
    out = []
    m = re.search(r"^uturns:\s*\n((?:[ \t]+-.*\n?)*)", text, flags=re.M)
    if not m:
        return out
    for line in m.group(1).splitlines():
        kv = dict(re.findall(r"(\w+):\s*([\w-]+)", line))
        if "node" in kv and "cond" in kv:
            out.append({"node": int(kv["node"]), "way": int(kv["way"]) if "way" in kv else None, "cond": kv["cond"]})
    return out


def uturn_cond(k, signal, m, arcs_by_id, survey, members):
    """합법 유턴 추정 (출처 estimate): 왕복 4차로 이상에서만 — 화살표 축 → left(좌회전 신호 시), 중간 횡단보도 → ped(보행 신호 시).
    경계 노드 → always(세상의 끝). 나머지 none(불법). route.yaml uturns:가 이긴다."""
    ain = arcs_by_id[m["from_arc"]]
    for sv in survey:
        if sv["node"] in members and (sv["way"] is None or sv["way"] == ain["way"]):
            return sv["cond"]
    if k == "boundary":
        return "always"
    if ain["lanes"] < 2 or ain["opp"] < 2:
        return "none"
    if k == "crosswalk":
        return "ped"
    if k == "junction" and signal:
        for ax in signal["axes"]:
            if m["from_arm"] in ax["arms"] and ax["plan"] in ("simul", "straightLeft"):
                return "left"
    return "none"


def junction_signal(i, arms, moves, surveyed, members, skip=frozenset()):
    """축 둘: 가장 마주 보는 팔 쌍(등급 높은 쪽 우선) = 주 축, 나머지 = 부 축. 운영은 노드 id로 섞는다.
    skip(우회전 전용 팔)은 어느 축에도 안 든다 — 늘 빨강, 정지 후 우회전."""
    n = len(arms)

    def pair_ok(x, y):
        # 같이 초록을 받을 수 있는 두 팔: 직진·우회전끼리 기하적으로 안 부딪힌다 (좌회전은 단계·비보호가 따로 다룬다)
        for i_, m in enumerate(moves):
            if m["from_arm"] != x or m["kind"] in ("left", "uturn"):
                continue
            for j in m.get("conflicts", []):
                o = moves[j]
                if o["from_arm"] == y and o["kind"] not in ("left", "uturn"):
                    return False
        return True

    cls = []
    for a_ in arms:
        ar = a_["in"] or a_["out"]
        cls.append(ar["cls"])
    best = None
    for x in range(n):
        for y in range(x + 1, n):
            if x in skip or y in skip:
                continue
            opp = abs(abs(math.degrees(wrap(arms[x]["b"] - arms[y]["b"]))) - 180)
            if not pair_ok(x, y) or not pair_ok(y, x):
                opp = 999  # 마주 보는 듯해도 진로가 겹치면 같이 못 간다
            rk = RANK.get(base_cls(cls[x]), 0) + RANK.get(base_cls(cls[y]), 0)
            key = (opp > 45, -rk, opp, x, y)
            if best is None or key < best[0]:
                best = (key, x, y)
    key, x, y = best
    main = [x, y] if not key[0] else [x]  # 마주 보지 않는 두 팔은 같이 초록을 받지 않는다
    rest = [k for k in range(n) if k not in main and k not in skip]
    a_cls = max((base_cls(cls[k]) for k in main), key=lambda c: RANK.get(c, 0))
    b_cls = max((base_cls(cls[k]) for k in rest), key=lambda c: RANK.get(c, 0)) if rest else a_cls
    link = any(c.endswith("_link") for c in cls)
    cyc, green = default_signal(a_cls, b_cls, link)
    sv = next((surveyed[m] for m in members if isinstance(m, int) and m in surveyed), None)
    if sv:
        cyc, green = int(sv.get("lunch_cycle", cyc)), int(sv.get("lunch_green", green))

    plans = ("simul", "straightLeft", "straight")
    def has_left(arm_ids):
        return any(m["kind"] == "left" and m["from_arm"] in arm_ids for m in moves)


    def two_lane(arm_ids):
        # 좌회전 화살표가 있는 신호는 최소 편도 2차로 (작성자 2026-10-24). 들어오는 호가 1차로인 팔이 있으면 직진 신호(비보호 좌회전)
        return all(arms[k]["in"] is None or arms[k]["in"]["lanes"] >= 2 for k in arm_ids)

    def pick(k, arm_ids):
        # 편도 1차로 → 직진 신호 (좌회전은 비보호). 편도 2차로 이상에서 좌회전이 있으면 화살표 신호 (비보호 좌회전 없음)
        if not two_lane(arm_ids):
            return "straight"
        if has_left(arm_ids):
            return ("simul", "straightLeft")[h32("plan", i, k) % 2]
        return plans[h32("plan", i, k) % 3]

    p1 = pick(0, main)
    if sv and sv.get("plan"):
        p1 = sv["plan"]
    s1, l1 = green, 0
    if p1 == "straightLeft":
        s1 = max(MIN_PHASE_REAL, round(green * STRAIGHT_SHARE_SL))
        l1 = max(MIN_PHASE_REAL, round(cyc * LEFT_PHASE_SHARE))
    rest_sec = cyc - s1 - l1
    if rest_sec < MIN_PHASE_REAL:
        cyc += MIN_PHASE_REAL - rest_sec
        rest_sec = MIN_PHASE_REAL
    axes = [{"arms": main, "plan": p1, "straight": s1, "left": l1, "unprotected_left": p1 == "straight" and has_left(main)}]
    # 나머지 팔: 한 축엔 마주 보는 두 팔까지만 (교차하는 팔이 같이 초록이면 상자 안에서 부딪힌다 — 5·6갈래).
    # 마주 보는 짝부터 묶고, 남는 팔은 혼자 한 단계 (내 방향 직·좌·우)
    groups, pool = [], list(rest)
    while pool:
        pair = None
        for x in range(len(pool)):
            for y in range(x + 1, len(pool)):
                opp = abs(abs(math.degrees(wrap(arms[pool[x]]["b"] - arms[pool[y]]["b"]))) - 180)
                if not pair_ok(pool[x], pool[y]) or not pair_ok(pool[y], pool[x]):
                    continue
                if opp <= 45 and (pair is None or opp < pair[0]):
                    pair = (opp, pool[x], pool[y])
        if pair:
            groups.append([pair[1], pair[2]])
            pool = [k for k in pool if k not in (pair[1], pair[2])]
        else:
            groups.append([pool.pop(0)])
    if groups:
        per = max(MIN_PHASE_REAL, rest_sec // len(groups))
        cyc = s1 + l1 + per * len(groups)
    for gi, g in enumerate(groups):
        p2 = pick(1 + gi, g) if len(g) == 2 else "simul"
        s2, l2 = per, 0
        if p2 == "straightLeft":
            l2 = max(25, round(per * 0.35))
            s2 = per - l2
            if s2 < 25:
                p2, s2, l2 = "simul", per, 0
        axes.append({"arms": g, "plan": p2, "straight": s2, "left": l2, "unprotected_left": p2 == "straight" and has_left(g)})
    return {"cycle": cyc, "axes": axes, "crosswalk": False}


# ───────────────────────── 합성 격자 (서울 기준선·도쿄 좌측통행) ─────────────────────────

def synthetic(pack_dir, spec, left):
    m = dict(kv.split("=") for kv in spec.split())
    gx, gy = (int(v) for v in m["grid"].split("x"))
    block = float(m["block"])
    lanes = int(m["lanes"])
    lat0, lon0 = 37.5, 127.0
    dlat = block / (R * math.pi / 180)
    dlon = block / (R * math.pi / 180 * math.cos(math.radians(lat0)))
    elements = []
    nid = 1
    grid = {}
    for y in range(-1, gy + 1):
        for x in range(-1, gx + 1):
            if (x in (-1, gx)) and (y in (-1, gy)):
                continue
            grid[(x, y)] = nid
            elements.append({"type": "node", "id": nid, "lat": lat0 + y * dlat, "lon": lon0 + x * dlon})
            nid += 1
    wid = 1000
    for (x, y), n in sorted(grid.items()):
        for dx, dy in ((1, 0), (0, 1)):
            o = grid.get((x + dx, y + dy))
            if o is None:
                continue
            inner = 0 <= x < gx and 0 <= y < gy and 0 <= x + dx < gx and 0 <= y + dy < gy
            elements.append({"type": "way", "id": wid, "nodes": [n, o],
                             "tags": {"highway": "tertiary" if inner else "unclassified", "lanes": str(2 * lanes)}})
            wid += 1
    snap = {"elements": elements, "osm3s": {"timestamp_osm_base": "synthetic"}, "query": f"synthetic {spec}"}
    import os
    os.makedirs(f"{pack_dir}/osm", exist_ok=True)
    json.dump(snap, open(f"{pack_dir}/osm/roads.overpass.json", "w"), sort_keys=True, separators=(",", ":"))
    margin = 0.25
    bb = [lon0 - (1 + margin) * dlon, lat0 - (1 + margin) * dlat, lon0 + (gx + margin) * dlon, lat0 + (gy + margin) * dlat]
    # 경계 노드(격자 밖 한 칸)는 bbox 밖이 되도록 bbox를 안쪽 격자 + 여유로
    bb = [lon0 - margin * dlon, lat0 - margin * dlat, lon0 + (gx - 1 + margin) * dlon, lat0 + (gy - 1 + margin) * dlat]
    return bb


def main():
    pack_dir = sys.argv[1]
    args = sys.argv[2:]
    if "--synthetic" in args:
        spec = args[args.index("--synthetic") + 1]
        bb = synthetic(pack_dir, spec, "--left" in args)
        rt = f"{pack_dir}/route.yaml"
        import os
        text = open(rt, encoding="utf-8").read() if os.path.exists(rt) else ""
        line = f"network_bbox: [{bb[0]:.7f}, {bb[1]:.7f}, {bb[2]:.7f}, {bb[3]:.7f}]  # 합성 격자 ({spec})"
        text = re.sub(r"^network_bbox:.*$", line, text, flags=re.M) if "network_bbox:" in text else \
            f"# 합성 격자 도로망 (tools/build_network.py --synthetic). 손으로 고치지 않는다.\n{line}\n" + text
        open(rt, "w", encoding="utf-8").write(text)
    snap, nodes, arcs, stats, side = build(pack_dir)
    out = {
        "license": "ODbL 1.0 — © OpenStreetMap contributors. 생성물: tools/build_network.py, 손으로 고치지 않는다."
        if snap.get("query", "").startswith("synthetic") is False else "합성 격자 (OSM 아님). 생성물: tools/build_network.py.",
        "osm_snapshot": snap["osm3s"]["timestamp_osm_base"],
        "query": snap.get("query", ""),
        "traffic_side": "left" if side < 0 else "right",
        "nodes": nodes,
        "arcs": arcs,
    }
    text = json.dumps(out, ensure_ascii=False, sort_keys=True, indent=1) + "\n"
    open(f"{pack_dir}/network.yaml", "w", encoding="utf-8").write(text)
    real = sum(a["len_real"] for a in arcs if a["twin"] is None or a["id"] < a["twin"])
    short = [a for a in arcs if a["len_real"] / TIME_SCALE - nodes[a["from"]]["box_half"] - nodes[a["to"]]["box_half"] < MIN_ARC_RIBBON_M
             and nodes[a["from"]]["kind"] != "boundary" and nodes[a["to"]]["kind"] != "boundary"]
    print(f"노드 {len(nodes)} (교차로 {sum(n['kind'] == 'junction' for n in nodes)} · 횡단보도 {sum(n['kind'] == 'crosswalk' for n in nodes)}"
          f" · 경계 {sum(n['kind'] == 'boundary' for n in nodes)} · 이음 {sum(n['kind'] == 'joint' for n in nodes)})"
          f" · 호 {len(arcs)} · 도로 실거리 {real / 1000:.2f} km = 리본 {real / TIME_SCALE / 1000:.2f} km · 신호 {stats['signal']}"
          f" · 동작 {stats['moves']} · 짧은 호 {len(short)}")
    print(f"경계 짧은 호 삭제 {stats['stubs_removed']} · 우회전 전용 팔 {stats['riro']}")
    print(f"유턴: 좌회전시 {stats.get('uturn_left', 0)} · 보행신호시 {stats.get('uturn_ped', 0)} · 상시 {stats.get('uturn_always', 0)}"
          f" · 불법 {stats.get('uturn_none', 0)}")
    arm_hist = {}
    for n in nodes:
        arm_hist[len(n["arms"])] = arm_hist.get(len(n["arms"]), 0) + 1
    print("팔 수 분포:", dict(sorted(arm_hist.items())))
    for w in stats["warn"][:20]:
        print("경고:", w)


if __name__ == "__main__":
    main()
