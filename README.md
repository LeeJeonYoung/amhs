# AGV Fleet — 미니 AMHS (3대 라인트레이서 함대)

에듀이노 라인트레이서 키트 3대 + nRF24L01 무선으로 만드는 **자동운반로봇(AGV) 함대**.
중앙 PC(Mac)가 격자 위에서 3대를 통합 제어하고, **교차로에서 좌/우/직진/유턴을 골라** 길을 내며,
충돌 없이 교통관리하고, 로컬 LLM이 자연어로 함대를 지휘한다. (반도체 팹의 AMHS를 장난감 스케일로 재현)

> **하드웨어가 없어도** 알고리즘·관제·LLM 전 스택을 노트북만으로 결정론적으로 시연/테스트할 수 있다.
> (`server/nav_demo.py`, `fleet_grid_server.py --sim`, `tests/` 18개 통과)

## 아키텍처 (3층)
```
┌─ Layer 3: 두뇌 (로컬 LLM / Ollama) ─────────────────┐
│  자연어 → 도구 호출  "1번 15번으로, 2번 12번으로"   │  brain/llm_grid.py (격자) / llm_orchestrator.py (8자)
├─ Layer 2: 작전참모 (Mac, Python) ───────────────────┤
│  경로계산·교차로 예약·데드락 회복·pose 추적          │  server/fleet_grid_server.py (격자) / fleet_server.py (8자)
│  교차로 회전결정 알고리즘(단일 정본)                 │  server/amhs/navigator.py
│        ↓ USB 시리얼                                  │
│  [허브 UNO + nRF24]  ))) 2.4GHz (((                  │  firmware/hub/hub.ino
├─ Layer 1: 반사신경 (로봇 UNO ×3) ───────────────────┤
│  라인추종 + 교차로감지 + 4방향 회전 (실시간)         │  firmware/robot_grid/robot_grid.ino (격자) / robot/robot.ino (8자)
└──────────────────────────────────────────────────────┘
```
- **LLM은 실시간 제어에 안 들어감** — 느려도 되는 '지휘'만. 안전·실시간은 아래 층이 책임.
- **8자 버전(검증 완료)은 보존**, 격자 버전을 별도 파일로 분리. 둘은 회전코드 통일.

## 핵심 — 교차로 회전결정 알고리즘
로봇은 자기가 몇 번 노드인지 모른다. "교차로 도착, 명령 대기"만 보고하고,
**어디로 틀지는 서버가 결정**해 내려보낸다. 결정은 순수 함수다:
```
회전 = (다음 노드 방위 − 현재 보는 방위) mod 4   →   STRAIGHT / RIGHT / UTURN / LEFT
```
`navigator.plan()` 이 최단경로(노드 열)를 이 회전들의 시퀀스로 펼치고, 펌웨어 mode 로 매핑한다.
→ `python3 server/nav_demo.py --scenarios` 로 즉시 눈으로 확인.

## 폴더
```
agv-fleet/
├─ README.md
├─ MANUAL.md                       D-Day 런북
├─ docs/
│  ├─ 00_기초지식_목차.md
│  ├─ 01_assembly.md  02_wiring.md  03_track.md
│  ├─ 04_run_order.md              8자 실행 순서
│  ├─ 05_calibration.md            부위별 캘리브레이션
│  ├─ 06_trial_and_error.md        ★ 시행착오 기록(면접 방어용)
│  └─ 07_grid_run_order.md         ★ 격자 실행 순서 (1→3대→LLM)
├─ firmware/
│  ├─ robot/robot.ino              8자 로봇(보존)
│  ├─ robot_grid/robot_grid.ino    ★ 격자 로봇(좌/우/직진/유턴)
│  ├─ hub/hub.ino                  허브(무선 게이트웨이)
│  └─ diagnostics/                 진단 5종(모터/IR/초음파/nRF핑/단독라인)
├─ server/
│  ├─ amhs/                        오프라인 두뇌 패키지
│  │  ├─ graph.py  geometry.py  router.py  dispatch.py  traffic.py  sim.py
│  │  └─ navigator.py              ★ 경로→회전명령 단일 정본 + 펌웨어 매핑
│  ├─ nav_demo.py                  ★ 회전결정 데모(하드웨어 0%)
│  ├─ fleet_grid_server.py         ★ 격자 관제(1~3대, 예약/회복, --sim 지원)
│  ├─ fleet_server.py              8자 관제(보존)
│  ├─ sim_grid.py                  오프라인 격자 시뮬 러너
│  └─ tests/test_amhs.py           18개 단위/통합 테스트
└─ brain/
   ├─ llm_grid.py                  LLM 사령관(격자) — 로컬 Ollama
   ├─ llm_grid_claude.py           ★ LLM 사령관(격자) — Claude API
   ├─ llm_grid_cli.py              ★ LLM 사령관(격자) — Claude 구독 CLI(-p 서브프로세스, API 키 불필요)
   └─ llm_orchestrator.py          LLM 사령관(8자, 출발/정지)
```

## 빠른 시작 (하드웨어 없이)
```bash
cd server
python3 nav_demo.py --scenarios            # 1) 회전결정 알고리즘 보기
python3 tests/test_amhs.py                 # 2) 전체 테스트 (18/18)
python3 fleet_grid_server.py --sim --robots 3   # 3) 가짜 로봇으로 3대 관제 시연
#   grid> goto 1 15   /  goto 2 12  /  goto 3 3  /  status
```
부품이 오면 격자는 **`docs/07_grid_run_order.md`**, 8자는 **`docs/04_run_order.md`** 순서대로.

## 통신 프로토콜 한눈에
- **무선(허브↔로봇)**: 허브가 라운드로빈으로 `Command{mode,speed}` 전송, 로봇이 ACK 페이로드로 `Status{id,state,node,obstacle}` 회신.
- **시리얼(PC↔허브)**: `C <id> <mode> <speed>` 송신 / `S <id> <state> <node> <obstacle>` 수신.
- **소켓(LLM/CLI↔서버)**: 격자는 JSON `{"action":"goto"|"stop"|"stop_all"|"status", ...}`.

## 모드 / 상태 값 (격자, `navigator.py` 정본)
- mode: `0 STOP / 1 RUN / 2 STRAIGHT / 3 RIGHT / 4 UTURN / 5 LEFT`
- state: `0 IDLE / 1 RUNNING / 2 WAIT_NODE / 3 NUDGE / 4 TURNING`
- 노드 id = `row*cols+col` (4x4면 0=좌상단…15=우하단), 방위 N=위 E=오른쪽 S=아래 W=왼쪽.

## 교통관리 / 데드락 (AMHS 핵심)
다음 노드 **예약**(한 노드 한 대) → 막히면 점유 노드 **회피 재경로** → 그래도 막히면 인접 빈 노드로
**유턴 후퇴**해 자기 노드를 비워 상대를 통과시킴. 같은 알고리즘을 오프라인 시뮬과 실물 관제가 공유한다.

## 확장 로드맵
- [x] 8자 → 격자(田): 경로 라우팅 + 교차로 회전결정 + 다중 교착 해소
- [x] 하드웨어 없는 결정론 테스트/시뮬 (18개 통과)
- [ ] 처리량/대기시간 로깅 → 포트폴리오용 데이터 수집
- [ ] 웹 대시보드 (로봇 위치 실시간 시각화)

> ⚠️ `docs/02_wiring.md` 핀 번호는 **키트 기본 배선 가정값**. 설명서와 대조해 맞추고,
> nRF24 가 쓰는 9~13 핀은 비워두세요. 회전 시간(`TURN_MS`/`UTURN_MS`)은 바닥/전압에 따라 재튜닝.
