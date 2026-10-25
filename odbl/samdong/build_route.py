"""동네 팩의 루프를 OSM 스냅샷에서 만든다 → road.yaml (생성물, ODbL).

    python3 -I tools/build_route.py cities/samdong

입력: <pack>/route.yaml (작성자: 웨이포인트·신호 실측), <pack>/osm/roads.overpass.json (tools/fetch_osm.sh).
출력: <pack>/road.yaml — JSON 문법(YAML의 부분집합). 손으로 고치지 않는다. 같은 입력 → 바이트 동일.
표준 라이브러리만 쓴다. 설계: docs/reviews/2026-10-23-samdong.md (0단계).
"""
import heapq
import json
import bisect
import math
import re
import sys

TIME_SCALE = 6.0          # SimConfig.timeScale. 실거리 → 리본 /6
CURVE_SEGMENT_M = 60.0    # SimConfig.curveSegmentM (리본)
CURVE_MAX = 0.012         # SimConfig.curveMax (리본, 1/m)
SMOOTH_REAL_M = 120.0     # 방위 평활 창 (실거리)
SNAP_M = 25.0             # 웨이포인트·신호 스냅 반경 (실거리)
MIN_GAP_RIBBON_M = 30.0   # 교차로 사이 최소 간격 (리본). 교차로 깊이 14m + 출발·도착 여유
R = 6371008.8

# 오토바이 통행 가능 규칙: daedongnamjido scripts/moto_build_graph.py의 규칙을 옮겨 적음
# (motorroad=yes·motorcycle=no·access=no|private 제외, oneway 존중). 동네 루프엔 trunk·motorway·service도 뺀다.
ALLOWED = {"primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link",
           "unclassified", "residential", "living_street"}
RANK = {"primary": 5, "secondary": 4, "tertiary": 3, "unclassified": 2, "residential": 1, "living_street": 0}

# 신호 추정값 (실제 초). 작성자 결정 2026-10-23: 실측 없이 한국 신호 운영 관행으로 추정, 신호는 반드시 있다.
# 근거: 도시부 신호 주기 대개 100~160초, 간선 쪽 초록이 길다, 보행 신호는 보행자 녹색 + 차량 녹색 70% 안팎.
# TODO(verify) 경찰청 「교통신호기 설치·관리 매뉴얼」의 주기 범위와 대조.
# (루프 도로 등급, 교차 도로 등급, 램프 여부) → (주기, 내 방향 초록). None = 비신호.
def default_signal(loop_cls, cross_cls, cross_is_link):
    a, b = RANK.get(loop_cls, 0), RANK.get(cross_cls, 0)
    if cross_is_link:
        return (120, 60)  # 나들목 접속부
    if a <= 1 and b <= 1:
        return None  # 골목 × 골목
    if a >= 5:
        return (150, 70) if b >= 3 else (130, 85)  # 간선: 큰길과는 길게, 골목과는 간선 쪽 초록이 길다
    if a >= 3 and b <= 1:
        return None  # 2차로 도로에 붙는 골목 입구: 한국 관행상 대개 비신호 (v1, 작성자 "신호는 있어야"는 큰길 교차로·횡단보도로 채운다)
    if a >= 3:
        return (120, 55) if b >= 3 else (100, 60)
    return (120, 40)  # 골목에서 큰길을 만남: 내 쪽 초록이 짧다


# 한국식 신호 운영 추정 (작성자 2026-10-24: "직진, 직진·좌회전 동시, 직진 정지-좌회전").
# 내가 좌회전하는 교차로엔 좌회전 신호가 반드시 있다. 나머지는 osm 노드 번호로 섞는다 (결정적).
TURN_DEG = 50                  # 앞뒤 방위 차가 이보다 크면 좌·우회전
LEFT_PHASE_SHARE = 0.2         # 직진 후 좌회전: 화살표 단계 = 주기 × 0.2 (최소 30초)
STRAIGHT_SHARE_SL = 0.65       # 직진 후 좌회전: 직진 초록 = 기존 초록 × 0.65

CROSSWALK_SIGNAL = (100, 75)   # 중간 횡단보도 보행 신호: 주기 100초, 차량 초록 75초
CROSSWALK_GAP_REAL_M = 330.0   # 신호 사이가 이보다 멀면 중간에 횡단보도 신호


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


def parse_route(text):
    """route.yaml의 작은 부분집합: `key: 값`, `key:` 아래 `  - {k: v, ...}` 목록. 주석(#) 허용."""
    out, cur = {}, None
    for raw in text.splitlines():
        line = re.sub(r"\s+#.*$", "", raw) if '"' not in raw else raw
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = re.match(r"^(\w+):\s*(.*)$", line)
        if m:
            key, val = m.group(1), m.group(2).strip()
            if val == "":
                out[key] = cur = []
            else:
                out[key] = scalar(val)
                cur = None
            continue
        m = re.match(r"^\s+-\s*\{(.*)\}\s*(#.*)?$", line)
        if m and cur is not None:
            cur.append({k: scalar(v) for k, v in re.findall(r'(\w+):\s*("(?:[^"\\]|\\.)*"|[^,]+)', m.group(1))})
            continue
        raise SystemExit(f"route.yaml: 읽을 수 없는 줄: {raw}")
    return out


def scalar(v):
    v = v.strip()
    if v.startswith('"'):
        return json.loads(v)
    if v in ("true", "false"):
        return v == "true"
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


def main(pack_dir):
    route = parse_route(open(f"{pack_dir}/route.yaml", encoding="utf-8").read())
    snap = json.load(open(f"{pack_dir}/osm/roads.overpass.json", encoding="utf-8"))
    nodes = {e["id"]: (e["lat"], e["lon"]) for e in snap["elements"] if e["type"] == "node"}
    ways = {}
    adj = {}
    node_ways = {}
    nbr = {}  # 방향 무시 이웃: 갈림길 판정용
    for e in snap["elements"]:
        if e["type"] != "way":
            continue
        t = e.get("tags", {})
        hw = t.get("highway", "")
        if hw not in ALLOWED or t.get("motorroad") == "yes" or t.get("motorcycle") == "no":
            continue
        if t.get("access") in ("no", "private") or t.get("area") == "yes":
            continue
        ways[e["id"]] = t
        ow = t.get("oneway", "no")
        if t.get("junction") == "roundabout" and ow != "no":
            ow = "yes"
        ns = [n for n in e["nodes"] if n in nodes]
        for n in ns:
            node_ways.setdefault(n, set()).add(e["id"])
        for a, b in zip(ns, ns[1:]):
            w = round(dist(nodes[a], nodes[b]) * 10)  # 정수 dm: 동률 처리까지 결정적
            pairs = [(a, b)] if ow in ("yes", "1", "true") else [(b, a)] if ow == "-1" else [(a, b), (b, a)]
            for u, v in pairs:
                adj.setdefault(u, []).append((w, v, e["id"]))
            nbr.setdefault(a, set()).add((b, e["id"]))
            nbr.setdefault(b, set()).add((a, e["id"]))
    for u in adj:
        adj[u].sort()

    def dijkstra(src, dsts):
        best = {src: 0}
        prev = {}
        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > best.get(u, 1 << 62):
                continue
            if u in dsts:
                path = [u]
                while path[-1] != src:
                    path.append(prev[path[-1]][0])
                path.reverse()
                return d, path, [prev[n][1] for n in path[1:]]
            for w, v, wid in adj.get(u, ()):
                nd = d + w
                if nd < best.get(v, 1 << 62) or (nd == best.get(v) and v in prev and u < prev[v][0]):
                    best[v] = nd
                    prev[v] = (u, wid)
                    heapq.heappush(pq, (nd, v))
        return None

    def seg_dist(p, a, b):
        # 국소 평면 근사 (수십 m 범위)
        kx = math.cos(math.radians(p[0])) * R * math.pi / 180
        ky = R * math.pi / 180
        ax, ay = (a[1] - p[1]) * kx, (a[0] - p[0]) * ky
        bx, by = (b[1] - p[1]) * kx, (b[0] - p[0]) * ky
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
        return math.hypot(ax + t * dx, ay + t * dy)

    def near_nodes(p):
        # 반경 안을 지나는 간선의 양끝 노드. 노드가 듬성한 긴 길도 잡힌다.
        out = set()
        for u, es in adj.items():
            for _, v, _ in es:
                if seg_dist(p, nodes[u], nodes[v]) <= SNAP_M:
                    out.update((u, v))
        return out

    wps = route["waypoints"]
    cands = []
    for i, wp in enumerate(wps):
        p = (wp["lat"], wp["lon"])
        c = sorted(near_nodes(p))
        if not c:
            raise SystemExit(f"웨이포인트 {i + 1} ({wp.get('note', '')}): {SNAP_M}m 안에 탈 수 있는 도로가 없다")
        cands.append(set(c))

    # 첫 웨이포인트의 후보마다 루프를 돌려 가장 짧은 것. 나머지 웨이포인트는 앞에서 가장 가까운 후보 (분리 도로면 진행 방향 차도가 잡힌다).
    best_loop = None
    for start in sorted(cands[0]):
        path, wids, total, cur = [start], [], 0, start
        ok = True
        for c in cands[1:] + [{start}]:
            r = dijkstra(cur, c)
            if r is None:
                ok = False
                break
            d, p, w = r
            total += d
            path += p[1:]
            wids += w
            cur = p[-1]
        if ok and (best_loop is None or total < best_loop[0]):
            best_loop = (total, path, wids)
    if best_loop is None:
        raise SystemExit("루프가 닫히지 않는다 (일방통행·통행 금지 도로를 확인)")
    _, path, wids = best_loop
    if len(set(path[:-1])) != len(path) - 1:
        seen, dup = set(), None
        for n in path[:-1]:
            if n in seen:
                dup = n
                break
            seen.add(n)
        trail = []
        for w in wids:
            nm = ways[w].get("name") or ways[w]["highway"]
            if not trail or trail[-1] != nm:
                trail.append(nm)
        raise SystemExit(f"루프가 같은 지점을 두 번 지난다 ({nodes[dup]}). 경로: {' → '.join(trail)}. 웨이포인트 순서를 확인")

    # 누적 실거리
    s_real = [0.0]
    for a, b in zip(path, path[1:]):
        s_real.append(s_real[-1] + dist(nodes[a], nodes[b]))
    loop_real = s_real[-1]

    # 교차로 후보: 루프의 앞뒤 간선이 아닌 다른 길이 닿는 노드
    cand = []
    for i in range(len(path) - 1):
        n = path[i]
        before = path[i - 1] if i > 0 else path[-2]
        after = path[i + 1]
        # 루프의 앞뒤 노드가 아닌 이웃으로 가는 길 = 갈림길 (같은 way가 교차로를 관통해도 잡힌다)
        others = sorted({w for v, w in nbr.get(n, ()) if v not in (before, after)})
        if not others:
            continue
        loop_cls = base_cls(ways[wids[i]]["highway"])
        cross = max(others, key=lambda w: (RANK.get(base_cls(ways[w]["highway"]), 0), not ways[w]["highway"].endswith("_link"), -w))
        cross_hw = ways[cross]["highway"]
        cand.append({"i": i, "node": n, "s": s_real[i], "loop": loop_cls, "cross": base_cls(cross_hw),
                     "link": cross_hw.endswith("_link")})

    # 작성자 실측 신호 스냅
    surveyed = {}
    for k, sg in enumerate(route.get("signals") or []):
        p = (sg["lat"], sg["lon"])
        near = [c for c in cand if dist(p, nodes[c["node"]]) <= SNAP_M]
        if not near:
            raise SystemExit(f"신호 {k + 1}: 루프 위 교차로 {SNAP_M}m 안에 없다")
        c = min(near, key=lambda c: (dist(p, nodes[c["node"]]), c["i"]))
        surveyed[c["i"]] = sg

    # 신호 교차로 고르기: 실측 > 등급. 리본에서 최소 간격 (루프 이음매 포함).
    for c in cand:
        sg = surveyed.get(c["i"])
        if sg:
            c["signal"] = {
                "lunch": {"cycle": sg["lunch_cycle"], "green": sg["lunch_green"]},
                "dinner": {"cycle": sg.get("dinner_cycle", sg["lunch_cycle"]), "green": sg.get("dinner_green", sg["lunch_green"])},
            }
            c["source"] = sg.get("source", "survey")
            c["prio"] = 100
        else:
            d = default_signal(c["loop"], c["cross"], c["link"])
            c["signal"] = None if d is None else {"lunch": {"cycle": d[0], "green": d[1]}, "dinner": {"cycle": d[0], "green": d[1]}}
            c["source"] = "estimate"
            c["prio"] = 0 if d is None else RANK.get(c["cross"], 0) + RANK.get(c["loop"], 0)
    gap_real = MIN_GAP_RIBBON_M * TIME_SCALE
    chosen = []
    for c in sorted((c for c in cand if c["signal"]), key=lambda c: (-c["prio"], c["s"])):
        if all(min(abs(c["s"] - o["s"]), loop_real - abs(c["s"] - o["s"])) >= gap_real for o in chosen):
            chosen.append(c)
    chosen.sort(key=lambda c: c["s"])
    # 신호 사이가 길면 중간 횡단보도 신호 (한국 도시부 관행). 루프 이음매 포함.
    crosswalks = []
    for idx, c in enumerate(chosen):
        nxt = chosen[(idx + 1) % len(chosen)]
        span = (nxt["s"] - c["s"]) % loop_real or loop_real
        n_extra = int(span // CROSSWALK_GAP_REAL_M)
        for m in range(1, n_extra + 1):
            pos = (c["s"] + span * m / (n_extra + 1)) % loop_real
            crosswalks.append({"i": -1 - len(crosswalks), "node": None, "s": pos, "loop": "crosswalk", "cross": "crosswalk",
                               "link": False, "kind": "crosswalk",
                               "signal": {"lunch": {"cycle": CROSSWALK_SIGNAL[0], "green": CROSSWALK_SIGNAL[1]},
                                          "dinner": {"cycle": CROSSWALK_SIGNAL[0], "green": CROSSWALK_SIGNAL[1]}},
                               "source": "estimate"})
    chosen = sorted(chosen + crosswalks, key=lambda c: c["s"])
    # 내 동작(직진·좌·우)과 신호 운영
    def pos_at(sr):
        sr %= loop_real
        j = max(0, bisect.bisect_right(s_real, sr) - 1)
        j = min(j, len(path) - 2)
        f = (sr - s_real[j]) / max(1e-9, s_real[j + 1] - s_real[j])
        a, b = nodes[path[j]], nodes[path[j + 1]]
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    for c in chosen:
        if c.get("kind") == "crosswalk":
            continue
        b_in = bearing(pos_at(c["s"] - 50), pos_at(c["s"] - 10))
        b_out = bearing(pos_at(c["s"] + 10), pos_at(c["s"] + 50))
        turn = math.degrees((b_out - b_in + math.pi) % (2 * math.pi) - math.pi)
        c["move"] = "left" if turn < -TURN_DEG else "right" if turn > TURN_DEG else "straight"
        sg = surveyed.get(c["i"])
        if sg and sg.get("plan"):
            c["plan"] = sg["plan"]
        elif c["link"]:
            c["plan"] = "straight"
        elif c["move"] == "left":
            c["plan"] = ("simul", "straightLeft")[c["node"] % 2]
        else:
            c["plan"] = ("simul", "straightLeft", "straight")[c["node"] % 3]
        if c["plan"] == "straightLeft" and not sg:
            for kind, t in c["signal"].items():
                cyc, g = t["cycle"], t["green"]
                left = max(30, round(cyc * LEFT_PHASE_SHARE))
                green = max(30, round(g * STRAIGHT_SHARE_SL))
                if cyc - green - left < 30:
                    green = max(30, cyc - left - 30)
                c["signal"][kind] = {"cycle": cyc, "green": green, "left": left}

    if "--debug" in sys.argv:
        trail = []
        for w in wids:
            nm = ways[w].get("name") or ways[w]["highway"]
            if not trail or trail[-1] != nm:
                trail.append(nm)
        print(f"loop {loop_real:.0f}m: {' → '.join(trail)}")
        for c in cand:
            print(f'{c["s"]:7.0f} {c["loop"]:>12} x {c["cross"]:<12} link={c["link"]} sig={c["signal"] is not None} {c.get("move", "")} {c.get("plan", "")} {nodes[c["node"]]}')
    if len(chosen) < 2:
        raise SystemExit(f"신호 교차로가 {len(chosen)}개뿐이다. 루프를 키우거나 신호 실측을 넣어라")
    # 루프 시작점(s=0)은 첫 신호 교차로 사이 한가운데로 옮긴다: 플레이어는 교차로 사이에서 출발한다.
    first, last = chosen[0]["s"], chosen[-1]["s"]
    shift = ((last + (loop_real - last + first) / 2) % loop_real)

    def rel(s):
        return (s - shift) % loop_real

    chosen_ids = {c["i"] for c in chosen}
    n_cross = sum(1 for c in chosen if c.get("kind") == "crosswalk")
    side = [c for c in cand if c["i"] not in chosen_ids]

    # 곡률: 10m 재표본 → 방위 → 120m 창 평활 → 리본 구간 경계마다 평균 곡률 (×6) → 클램프
    step = 10.0
    samples = []
    j = 0
    s = 0.0
    while s < loop_real:
        while s_real[j + 1] < s:
            j += 1
        f = (s - s_real[j]) / max(1e-9, s_real[j + 1] - s_real[j])
        a, b = nodes[path[j]], nodes[path[j + 1]]
        samples.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
        s += step
    m = len(samples)
    head = [bearing(samples[i], samples[(i + 1) % m]) for i in range(m)]
    # 펼친 방위 (연속)
    unwrapped = [head[0]]
    for h in head[1:]:
        d = (h - unwrapped[-1] + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + d)
    total_turn = unwrapped[-1] - unwrapped[0] + ((head[0] - head[-1] + math.pi) % (2 * math.pi) - math.pi)

    def theta(i):
        q, r = divmod(i, m)
        return unwrapped[r] + q * total_turn

    half = int(SMOOTH_REAL_M / step / 2)
    seg_real = CURVE_SEGMENT_M * TIME_SCALE
    n_seg = max(1, round(loop_real / seg_real))
    seg_real = loop_real / n_seg  # 루프가 구간으로 딱 나뉘게
    curves = []
    clamped = 0
    for k in range(n_seg):
        center = (k * seg_real + shift) / step
        i0 = int(round(center - seg_real / step / 2))
        i1 = int(round(center + seg_real / step / 2))
        # 구간 평균 곡률 = 구간 양끝(평활) 방위 차 / 길이
        def smooth(i):
            return sum(theta(i + d) for d in range(-half, half + 1)) / (2 * half + 1)
        k_real = (smooth(i1) - smooth(i0)) / ((i1 - i0) * step)
        k_rib = k_real * TIME_SCALE
        if abs(k_rib) > CURVE_MAX:
            clamped += 1
            k_rib = math.copysign(CURVE_MAX, k_rib)
        curves.append(round(k_rib, 5))

    def ll(n):
        return [round(nodes[n][0], 7), round(nodes[n][1], 7)]

    order = sorted(range(len(path) - 1), key=lambda i: rel(s_real[i]))
    polyline = [ll(path[i]) + [round(rel(s_real[i]), 1)] for i in order]
    out = {
        "license": "ODbL 1.0 — © OpenStreetMap contributors. 생성물: tools/build_route.py, 손으로 고치지 않는다.",
        "osm_snapshot": snap["osm3s"]["timestamp_osm_base"],
        "query": snap["query"],
        "loop_real_m": round(loop_real, 1),
        "curve_segment_real_m": round(seg_real, 1),
        "curves": curves,
        "intersections": [
            {"s_real": round(rel(c["s"]), 1), "osm_node": c["node"], "kind": c.get("kind") or f'{c["loop"]}x{c["cross"]}',
             "signal": c["signal"], "source": c["source"],
             **({"move": c["move"], "plan": c["plan"]} if "move" in c else {})}
            for c in sorted(chosen, key=lambda c: rel(c["s"]))
        ],
        "side_streets": [{"s_real": round(rel(c["s"]), 1), "kind": f'{c["loop"]}x{c["cross"]}'} for c in sorted(side, key=lambda c: rel(c["s"]))],
        "polyline": polyline,
    }
    text = json.dumps(out, ensure_ascii=False, sort_keys=True, indent=1) + "\n"
    open(f"{pack_dir}/road.yaml", "w", encoding="utf-8").write(text)

    gaps = []
    ss = [rel(c["s"]) for c in chosen]
    for a, b in zip(ss, ss[1:] + [ss[0] + loop_real]):
        gaps.append((b - a) / TIME_SCALE)
    names = []
    for w in wids:
        nm = ways[w].get("name") or ways[w]["highway"]
        if not names or names[-1] != nm:
            names.append(nm)
    print(f"루프 {loop_real:.0f}m 실거리 = 리본 {loop_real / TIME_SCALE:.0f}m · 신호 {len(chosen)} (그중 횡단보도 {n_cross}) · 옆길 {len(side)}")
    print(f"신호 간격 (리본 m): 평균 {sum(gaps) / len(gaps):.0f} · 최소 {min(gaps):.0f} · 최대 {max(gaps):.0f}")
    print(f"곡률 구간 {len(curves)} (클램프 {clamped}) · 실측 신호 {len(surveyed)} · 경로: {' → '.join(names)}")
    print("motorroad 접촉 0 (그래프에서 제외됨)")


if __name__ == "__main__":
    main(sys.argv[1])
