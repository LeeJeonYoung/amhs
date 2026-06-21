# 07. 격자(田) 버전 실행 순서 — 1대 → 3대 → LLM

> 8자(`04_run_order.md`)가 끝났다는 전제. 격자는 "교차로에서 좌/우/직진/유턴을 골라야"
> 하므로 `robot_grid.ino` + `fleet_grid_server.py` + `navigator.py` 세트를 쓴다.
> **하드웨어가 없어도 STEP 0·1 은 노트북만으로 시연·검증된다.**

## STEP 0 — 알고리즘만 먼저 (하드웨어 0%)
```bash
cd server
python3 nav_demo.py --scenarios          # 출발/도착 → 노드별 좌/우/직진/유턴 결정 출력
python3 nav_demo.py --from 0 --to 15 --heading N
python3 sim_grid.py --ab                 # 배차×경로 4조합 KPI 비교(오프라인 시뮬)
python3 tests/test_amhs.py               # 18개 단위/통합 테스트 (회전·예약·관제 전부)
```
- **확인**: 회전 결정이 말이 되는지(북 보고 동쪽 = 우회전), 시뮬 완료율 0.9+·충돌 0.

## STEP 1 — 가짜 로봇으로 관제 서버 전체 구동 (하드웨어 0%)
```bash
python3 fleet_grid_server.py --sim --robots 3
# grid> goto 1 15
# grid> goto 2 12
# grid> goto 3 3
# grid> status
```
- **확인**: 3대가 교차 목적지로 가며 로그에 `[재경로]`/`[회복]` 이 뜨고, 모두 `[완료]`.
- 이 단계가 통과하면 **제어 로직은 끝**. 이후는 같은 로직에 진짜 로봇을 붙이는 일.

## STEP 2 — 실물 1대 격자 주행
- [ ] `firmware/robot_grid/robot_grid.ino` 업로드 (ROBOT_ID=1). 8자용 `robot.ino` 와 별개.
- [ ] 십자(+) 또는 작은 격자 트랙. 라인 폭 19mm 검정테이프, 밝은 바닥.
- [ ] ⚠️ **초기 방향 규약**: 서버는 모든 로봇이 처음에 **북(N=행 감소 방향, 0번 노드 쪽)** 을 보고
      있다고 가정한다(`RobotState heading=N` 기본값). 로봇을 트랙에 올릴 때 **북을 향하게** 놓을 것.
      안 그러면 출발 노드에서의 첫 회전 명령이 어긋난다(직진해야 할 걸 좌회전 등). 다른 방향이 편하면
      `fleet_grid_server.py` 의 `RobotState(start)` 초기 heading 을 실제 방향으로 바꿔라.
- [ ] 허브 연결 후: `python3 fleet_grid_server.py --port /dev/tty.usbserial-XXXX --robots 1`
- [ ] `goto 1 3` → 로봇이 경로를 따라 교차로마다 서버 명령대로 좌/우/직진 하는지.
- [ ] **안 맞으면**: (먼저 초기 방향 규약 확인) 회전각 `TURN_MS`, 교차로 진입 `NUDGE_MS`, 재검출 `RENODE_MS` 튜닝.

## STEP 3 — 실물 3대 + 교차로 양보
- [ ] 로봇2/3 도 `robot_grid.ino`(ROBOT_ID=2,3) 업로드 + 배선.
- [ ] `python3 fleet_grid_server.py --port ... --robots 3`
- [ ] 서로 교차하는 목적지를 줘서(`goto 1 15`,`goto 2 12`) **한 노드에 한 대만** 들어가는지,
      막히면 우회/후퇴로 풀리는지 확인. (로그가 STEP 1 과 같은 모양이어야 정상)

## STEP 4 — LLM 사령관 연결 (두뇌 3종 중 택1)
fleet_grid_server 실행 중인 상태에서 새 터미널:
```bash
# (A) 로컬 Ollama — 오프라인·무료, 작은 모델
ollama serve & ; python3 brain/llm_grid.py
# (B) Claude API — 똑똑함, 종량 과금
export ANTHROPIC_API_KEY=... ; python3 brain/llm_grid_claude.py
# (C) Claude 구독 CLI — 똑똑함 + 구독 한도 내, API 키 불필요(권장)
python3 brain/llm_grid_cli.py        # 이미 로그인된 claude CLI 를 -p 로 호출
```
지시 예: `1번 12번에서 3번으로 운반, 2번은 15번으로` / `상태` / `전부 정지`
- **확인**: 자연어가 `assign_mission(1,12,3)`/`send_robot_to(2,15)` 도구 호출로 바뀌어 서버에 꽂히는지.

## STEP 5 — A→B 운반 작업 배차(시뮬·실물 공통)
```bash
# grid> 단건 운반:  mission <로봇> <적재> <하역>
grid> mission 1 12 3
# grid> 여러 작업 Hungarian 최적 배차:  dispatch <s1> <d1> <s2> <d2> ...
grid> dispatch 1 13 2 14
```
- 로봇이 적재지로 가 `[적재]` 로그 → 하역지로 가 `[하역]` 로그까지 나오면 정상.
- `dispatch` 는 유휴 로봇에 '가장 가까운 적재지' 기준으로 작업을 최적 할당(오프라인 시뮬과 동일 알고리즘).

## 명령/상태 값 (격자 버전, navigator.py 정본)
- mode: `0 STOP / 1 RUN / 2 STRAIGHT / 3 RIGHT / 4 UTURN / 5 LEFT`
- state: `0 IDLE / 1 RUNNING / 2 WAIT_NODE / 3 NUDGE / 4 TURNING`
- 노드 id = `row * cols + col` (4x4면 0=좌상단 … 15=우하단). 방위 N=위 E=오른쪽 S=아래 W=왼쪽.

## 흔한 문제 (격자 한정)
| 증상 | 원인/해결 |
|---|---|
| 출발 노드에서 안 나감 | 출발 명령이 회전(2~5)으로 가야 NUDGE 탈출 — 서버가 자동 처리. 안 되면 `goto` 재전송 |
| 회전 각도 안 맞음 | `TURN_MS`(90°)/`UTURN_MS`(180°) 실측 튜닝 |
| 회전 직후 같은 노드 재인식 | `RENODE_MS` ↑ |
| 3대 중 한 대가 계속 대기 | 정상(양보 중)일 수 있음. 오래면 `status` 로 occupied 확인, 데드락이면 회복 로그 확인 |
| 서버가 위치를 잘못 앎 | 로봇이 교차로를 놓침(인식 실패) → B/C 항목(`06_trial_and_error.md`) 재튜닝 |
