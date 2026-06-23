# web-sim — mini-AMHS 알고리즘 시뮬레이터 (JS)

천장 레일 위 다중 OHT 반송을 **배차(greedy / hungarian) × 경로(static / congestion-aware)** 로 비교하는 시뮬레이터.

`server/amhs/`(Python)가 **실물 격자 제어로 가는 다리**(노드 예약·데드락 회복·회전명령)라면,
이쪽(JS)은 **알고리즘 효과를 눈으로 보고(데모) 통계로 재는(하네스)** 용도다.

## 파일별 실행 / 동작

| 파일 | 실행 | 무엇을 하나 | 화면? |
|---|---|---|---|
| `index.html` | 브라우저로 열기 | **실시간 시각화 데모.** 격자·차량·간선 혼잡 히트맵을 캔버스로 그리고, 배차/경로/부하/속도를 슬라이더로 바꿔 KPI 변화를 즉시 본다. | ✅ 시각화 |
| `sim/harness.mjs` | `node sim/harness.mjs` | **헤드리스 실험 하네스(콘솔).** index.html과 동일한 시뮬 코어를 10 seed × 4 config × 2 부하로 돌려 마크다운 표 출력. 헤드라인 수치(cycle −49.8% 등) 생성 + 현실 변수 4종(고장·배터리·가변처리·MES 버스트)에서도 우위 유지 검증. | ❌ stdout |
| `sim/physical.mjs` | `node sim/physical.mjs` | **추상(BPR) vs 물리(노드점유+하드 blocking+데드락) 모델 비교.** 차량 밀도를 올리며 처리량·완료율·데드락 측정. | ❌ stdout |

## 핵심 모델 (메소드)
- **그래프**: rows×cols 격자, `dijkstra`(최단경로, 혼잡 가중치 옵션), `allPairs`(배차 비용용 전쌍 거리).
- **배차**: `dispatch()` — greedy(근접 우선) vs **`hungarian`(Kuhn–Munkres 손구현, O(n³))** = 차량×작업 비용행렬의 전역 최소 매칭. HOT 가산점 + 대기시간 가중.
- **경로**: `route()` — static(최단) vs congestion-aware(간선 가중치 = 길이 × (1 + α·동시주행대수)).
- **혼잡**: `startHop()` — BPR식 주행시간 증가 `tt = ff × (1 + β·load)`. 데드락을 추상화로 분리해 **알고리즘 효과만** 깨끗이 측정.
- **작업**: `genTasks()` — Poisson 도착, HOT 우선, `candidates()`가 window 30 후보만 추림.
- **루프**: `step()` = genTasks → 이동/처리 → dispatch → KPI. `kpis()`가 throughput/cycle/queue/util/congestion 산출.

## Python 시뮬(`server/amhs/`)과의 관계 — 같은 문제, 다른 질문
| | 이쪽 (JS) | server/amhs (Python) |
|---|---|---|
| 답하는 질문 | "어떤 알고리즘이 더 빠른가?" | "그 알고리즘을 실물 격자에서 충돌·교착 없이 돌릴 수 있나?" |
| 혼잡 처리 | BPR 슬로다운(추상, 데드락 없음) | 노드 예약(zone-lock) + 하드 blocking + 데드락 회복 |
| 실물 연결 | 없음(알고리즘 검증·시각화) | 회전명령(직진/좌/우/유턴) 생성 → 펌웨어로 직행 |
| 산출 | 시각 데모 + 헤드라인 % | deadlocks·reroutes·block_ratio + 자동화 테스트 |

> 요약: JS로 **알고리즘 우위를 증명**하고, Python으로 **실물 제약(데드락·예약)을 견디는지 검증**한 뒤 실제 로봇으로 옮겼다.
