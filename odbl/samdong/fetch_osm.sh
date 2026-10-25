#!/usr/bin/env bash
# OSM 스냅샷 받기 (Overpass, 1회). 결과는 커밋한다 — 런타임 호출 없음. 데이터 © OpenStreetMap contributors, ODbL.
#   tools/fetch_osm.sh cities/samdong "37.400,127.185,37.430,127.215"
set -euo pipefail
dir="$1"; bbox="$2"
q="[out:json][timeout:60];way[\"highway\"~\"^(trunk|trunk_link|primary|primary_link|secondary|secondary_link|tertiary|tertiary_link|unclassified|residential|living_street)$\"]($bbox);(._;>;);out body;"
mkdir -p "$dir/osm"
curl -sf -A "baedalai-pack-builder (sdongworks)" --data-urlencode "data=$q" https://overpass-api.de/api/interpreter \
  | python3 -I -c 'import json,sys; d=json.load(sys.stdin); d["query"]=sys.argv[1]; json.dump(d,sys.stdout,ensure_ascii=False,sort_keys=True,separators=(",",":"))' "$q" \
  > "$dir/osm/roads.overpass.json"
python3 -I -c 'import json,sys; d=json.load(open(sys.argv[1])); print(d["osm3s"]["timestamp_osm_base"], len(d["elements"]), "elements")' "$dir/osm/roads.overpass.json"
