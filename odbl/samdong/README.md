# 배달아이 — 삼동 도로망 파생 데이터 (ODbL 1.0)

이 폴더는 게임 "배달아이"가 앱 안에 담은 OpenStreetMap 파생 데이터베이스와, 그것을 만든 원본·도구입니다.
ODbL 1.0 §4.6에 따라 공개합니다.

지도 데이터 © OpenStreetMap contributors — https://www.openstreetmap.org/copyright
라이선스: Open Database License (ODbL) 1.0 — https://opendatacommons.org/licenses/odbl/1-0/

## 파일
| 파일 | 무엇 |
|---|---|
| `roads.overpass.json` | OSM 원본 스냅샷 (Overpass 응답, 쿼리와 시각 포함) |
| `route.yaml` | 루프 웨이포인트·신호 추정 입력 (사람이 쓴 것) |
| `road.yaml` | 루프 생성물 (`build_route.py`) |
| `network.yaml` | 도로망 생성물 (`build_network.py`) — 앱에 들어 있는 파생 DB |
| `build_route.py`, `build_network.py`, `fetch_osm.sh` | 생성 도구 |

## 다시 만들기
```
sh fetch_osm.sh samdong            # 원본 스냅샷 (선택 — 여기 있는 것을 써도 된다)
python3 -I build_route.py .        # road.yaml
python3 -I build_network.py .      # network.yaml
```
(레포 안에선 `cities/samdong/` 경로 기준. 여기선 같은 폴더에 둔 상태.)

이 데이터베이스를 고치거나 다시 배포하면 같은 ODbL 1.0으로 공개해야 합니다.
게임 화면(지도 그림)은 Produced Work로, 출처 표기만으로 족합니다.
