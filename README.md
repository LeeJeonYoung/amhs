# amhs — 미니 AMHS (다중 OHT 반송 시뮬레이터 + 라인트레이서 실물)

반도체 팹의 **자동반송시스템(AMHS)** 을 장난감 스케일로 재현한 프로젝트.
다수 OHT를 **어디에 배차하고(스케줄링) 혼잡 속 어떤 경로로 보낼지(탐색)** 실시간 최적화하는 문제를
**시뮬레이션으로 증명하고 → 라인트레이서 실물로 검증**했다.

| 구성 | 위치 | 역할 |
|---|---|---|
| 알고리즘 시뮬 (JS) | `web-sim/` | 배차·경로 A/B 비교 + 브라우저 실시간 시각화 |
| 격자 제어 시뮬 (Python) | `server/amhs/` | 노드 예약·데드락 회복·회전명령 — 실물 제어로 직결 |
| 실물 | `firmware/` | 라인트레이서 + nRF24 무선 + 중앙 허브 (1대 격자 완주) |
| LLM 지휘 | `brain/` | 자연어 → 행동(JSON). 실시간 루프엔 미투입 |

> **하드웨어가 없어도** 알고리즘·관제·LLM 전 스택을 노트북만으로 결정론적으로 시연/테스트할 수 있다.

---

## 결과 (동일 시드 A/B · seed 10개 평균)
| 부하 | 지표 | `greedy+static` → `hungarian+congestion` |
|---|---|---|
| 중부하 | 평균 사이클 타임 | **−49.8%** |
| 중부하 | 평균 대기시간 | **−82.6%** |
| 포화 | 처리량 | **+19.1%** |

- 혼잡은 **BPR식**(통행량↑→주행시간↑)으로 추상화해, 하드 블로킹의 데드락이 알고리즘 비교를 흐리지 않게 **효과만 분리** 측정.
- 배차 **헝가리안(Kuhn–Munkres)** 은 외부 라이브러리 없이 직접 구현.
- 재현: `node web-sim/sim/harness.mjs` (헤드라인 수치) · `python3 server/overnight_sim.py` (격자 스윕)

## ★ 실물 검증 (2026-06-23)
라인트레이서 **1대로 격자 경로 완주** — 라인추종 → 교차로 감지 → **연속 회전** → 목적지 정지 실주행 성공.
시뮬(알고리즘)을 실물 제어로 내리며 마주친 물리 계층 디버깅(IR 센서 극성·조향 방향·회전 기구학·자력출발 토크·길잃음 복구·전원/무선 안정성) → [`docs/08_physical_bringup.md`](docs/08_physical_bringup.md)
**여러 대 동시 제어는 통신·전원 안정성이 병목이라 진행 중** — 다중 로봇의 병목은 알고리즘이 아니라 전원·통신 안정성(실 AMHS와 동일).

---

## 아키텍처 (3층)
```
┌─ Layer 3: 두뇌 (LLM) ───────────────────────────────┐
│  자연어 → 행동(JSON)  "1번 15번으로, 2번 12번으로"   │  brain/llm_grid*.py
├─ Layer 2: 관제 (Mac · Python) ──────────────────────┤
│  배차·경로·교차로 예약·데드락 회복·pose 추적         │  server/fleet_grid_server.py
│  교차로 회전결정 (단일 정본)                         │  server/amhs/navigator.py
│        ↓ USB 시리얼                                  │
│  [허브 UNO + nRF24]  ))) 2.4GHz (((                  │  firmware/hub/hub.ino
├─ Layer 1: 반사신경 (로봇 UNO) ──────────────────────┤
│  라인추종 + 교차로감지 + 4방향 회전 (실시간)         │  firmware/robot_grid/robot_grid.ino
└──────────────────────────────────────────────────────┘
```
- **LLM은 실시간 제어에 안 들어간다** — 느려도 되는 '지휘'만. 안전·실시간 제어는 결정론 알고리즘이 책임.
- LLM은 자연어를 **작업·우선순위 같은 입력**으로 바꿀 뿐, 경로·예약·충돌 회피 같은 실행은 알고리즘이 그대로 한다.

## 소스 구조
```
amhs/
├─ web-sim/                         # 알고리즘 시뮬 (JS) — 시각화 + 헤드라인 수치
│  ├─ index.html                    #   브라우저 실시간 시각화 데모 (배차/경로/부하 토글)
│  └─ sim/
│     ├─ harness.mjs                #   10-seed 헤드리스 하네스 (−49.8%/−82.6%/+19.1% 생성)
│     └─ physical.mjs               #   추상(BPR) vs 물리(예약+데드락) 모델 비교
│
├─ server/                          # 중앙 관제 + 격자 시뮬 (Python)
│  ├─ amhs/                         #   ★ 시뮬레이션 코어 (알고리즘 두뇌)
│  │  ├─ graph.py                   #     격자 그래프 + 다익스트라 + all_pairs 캐시
│  │  ├─ dispatch.py                #     배차: hungarian(전역 최적) / greedy(근시안)
│  │  ├─ router.py                  #     경로: static(최단) / congestion(BPR 혼잡)
│  │  ├─ traffic.py                 #     노드 예약(zone-lock) — 한 노드 1대, 충돌 0
│  │  ├─ geometry.py                #     방위↔회전명령  (목표−현재) mod 4
│  │  ├─ navigator.py               #     경로 → 회전명령 시퀀스 (펌웨어 매핑 단일 정본)
│  │  ├─ sim.py                     #     GridAMHS — step 루프 + KPI 산출
│  │  └─ timing.py · rng.py         #     타이밍 / 시드 기반 RNG(재현성)
│  ├─ sim_grid.py                   #   격자 시뮬 단발 러너
│  ├─ overnight_sim.py              #   파라미터 스윕(배차×경로×그리드×시드)
│  ├─ compare_routing.py · scale_test.py
│  ├─ fleet_grid_server.py          #   실물/시뮬 관제 서버 (--sim 가짜로봇 지원)
│  ├─ grid_drive.py                 #   실물 고수준 경로 시퀀서 ("S1 R1 R1 R1" + 재전송)
│  ├─ serial_broker.py              #   허브 시리얼 공유(여러 프로세스가 한 포트)
│  └─ tests/test_amhs.py            #   자동화 테스트 18개 (충돌0·데드락 회복 검증)
│
├─ firmware/                        # 아두이노 펌웨어 (실물)
│  ├─ robot_grid/robot_grid.ino     #   차량: 라인추종 + 교차로 감지 + 좌/우/직진/유턴
│  ├─ hub/hub.ino                   #   nRF24 무선 허브(게이트웨이)
│  ├─ diagnostics/                  #   진단 스케치(모터/IR/초음파/nRF핑/단독 라인추종)
│  └─ _versions/                    #   펌웨어 진화 v1~v10 (시행착오 보존)
│
├─ brain/                          # LLM 지휘 (자연어 → 행동)
│  ├─ llm_grid.py                   #   로컬 LLM(Ollama) 도구 호출
│  ├─ llm_grid_claude.py            #   Anthropic API (수동 도구 루프)
│  └─ llm_grid_cli.py               #   Claude 구독 CLI 서브프로세스(API 키 불필요)
│
├─ docs/                           # 조립·배선·트랙·캘리브레이션·트러블슈팅·기초지식
├─ MANUAL.md                       # 실물 D-Day 런북
└─ README.md
```

## 빠른 시작 (하드웨어 없이)
```bash
# 1) 알고리즘 시뮬 — 브라우저에서 눈으로
open web-sim/index.html                       # 배차/경로/부하 토글하며 KPI 관찰
node web-sim/sim/harness.mjs                  # 헤드라인 수치(−49.8% 등) 재현

# 2) 격자 제어 시뮬 — 예약/데드락/회전명령
cd server
python3 nav_demo.py --scenarios              # 회전결정 알고리즘 보기
python3 -m pytest tests/test_amhs.py         # 전체 테스트 (18개)
python3 fleet_grid_server.py --sim --robots 3   # 가짜 로봇 3대 관제 시연
#   grid> goto 1 15  /  goto 2 12  /  status
```
부품이 오면 격자 실행 순서는 [`docs/07_grid_run_order.md`](docs/07_grid_run_order.md).

## 핵심 알고리즘 한눈에
- **교차로 회전결정** — 로봇은 자기 노드를 모른다. "교차로 도착, 명령 대기"만 보고하고 서버가 결정:
  `회전 = (다음 노드 방위 − 현재 보는 방위) mod 4 → STRAIGHT / RIGHT / UTURN / LEFT`. 이 명령을 펌웨어가 그대로 실행.
- **배차** — `hungarian`이 차량×작업 비용행렬의 전역 최소 매칭(근시안 greedy 제거). 비용 = 거리 − 대기가중 − HOT 보너스. 윈도우(상위 30개 작업)만 후보로 둬 규모를 제한.
- **교통/데드락** — 다음 노드 **예약**(한 노드 1대) → 막히면 점유 노드 **회피 재경로** → 그래도 막히면 인접 빈 노드로 **유턴 후퇴**해 길을 비움. 같은 로직을 오프라인 시뮬과 실물 관제가 공유.

## 통신 프로토콜
- **무선(허브↔로봇)**: 허브가 라운드로빈으로 `Command{mode,speed}` 전송, 로봇이 ACK로 `Status{id,state,node,obstacle}` 회신.
- **시리얼(PC↔허브)**: `C <id> <mode> <speed>` 송신 / `S <id> <state> <node> <obstacle>` 수신.
- **소켓(LLM↔서버)**: JSON `{"action":"goto"|"stop"|"stop_all"|"status", ...}`.
- mode: `0 STOP / 1 RUN / 2 STRAIGHT / 3 RIGHT / 4 UTURN / 5 LEFT` · 노드 id = `row*cols+col` · 방위 N=위 E=오른쪽 S=아래 W=왼쪽.

---
> ⚠️ 배선 핀 번호는 키트 기본 배선 가정값 — 설명서와 대조해 맞추고, nRF24가 쓰는 9~13핀은 비워둘 것. 회전 시간(`TURN_MS`)은 바닥/전압에 따라 재튜닝.
