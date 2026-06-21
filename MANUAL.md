# AGV 함대 — 부품 도착 D-Day 매뉴얼 (한 장으로 따라가기)

> 위에서 아래로 순서대로. 각 단계의 **[확인]** 이 끝나야 다음으로 넘어간다.
> 더 자세한 배경은 `docs/` 와 `README.md`, 부위 캘리브레이션은 `docs/05_calibration.md`.

---

## STEP 0 — 부품 기다리는 동안 (미리)
- [ ] 아두이노 IDE 설치
- [ ] 라이브러리 매니저 → **"RF24" by TMRh20** 설치
- [ ] `cd server && pip install -r requirements.txt`
- [x] (LLM용) Ollama + `qwen3:1.7b` 모델 — 이미 설치됨 (`ollama list` 로 확인). 파이썬 패키지(pyserial/ollama)도 설치 완료

## STEP 1 — 개봉 & 부품 확인
- [ ] 라인트레이서 키트 ×3, nRF24 모듈 ×6, AMS1117 어댑터, 허브용 UNO ×1
- [ ] 점퍼선(암-수), AA 건전지, 검정 절연테이프(19mm)
- [ ] **[확인]** nRF24용 D9~D13 핀이 키트 기본 배선과 겹치지 않는지 설명서 대조 (`docs/02_wiring.md`)

## STEP 2 — 로봇 1대 조립
- [ ] 키트 1대 제조사 설명서대로 조립 (`docs/01_assembly.md`)
- [ ] 부품 연결 — **처음이면 `docs/02_쉬운배선설명.md`** (그림+설명), 요약표는 `docs/02_wiring.md`
- [ ] 특히 **무전기(nRF24)** 는 직접 배선: VCC=5V / CE9·CSN10·SCK13·MOSI11·MISO12
- [ ] **[확인]** 전원 넣기 전 점검 체크(02_쉬운배선설명 §7), 그 뒤 전원 인가 시 보드 LED 점등

## STEP 3 — 부위별 진단 + 캘리브레이션  ★도착 즉시 핵심
스케치: `firmware/diagnostics/` (자세한 판정은 `docs/05_calibration.md`)

| # | 업로드 | 본다 | 기록 |
|---|--------|------|------|
| 1 | `01_motor_test` | 바퀴 들고 좌/우 방향 | 방향 정상 / IN 스왑 |
| 2 | `02_ir_test` | 검정 라인 위 L/R 값 | **LINE_ON** = HIGH/LOW |
| 3 | `03_sonar_test` | 거리 cm | **OBSTACLE_CM** |

- [ ] 모터 방향 확인 — 기대출력: `LEFT forward` 일 때 왼바퀴 전진
- [ ] IR 극성 확인 — 기대출력: `L=1 R=1 -> 양쪽 검정 = 교차로`
- [ ] 초음파 확인 — 기대출력: `dist = NN cm` 가 손 거리 따라 변함
- [ ] **[확인]** 위 값들을 `robot.ino` 에 반영 (line 36 / 41 / 27~30 — 표는 05_calibration.md)

## STEP 4 — 1대 단독 라인추종 (무선 없이)
- [ ] `firmware/diagnostics/05_linefollow_solo` 업로드
      (⚠️ `robot.ino` 는 무선 RUN 을 받아야 움직이므로 단독 검증엔 이 스케치를 쓴다)
- [ ] 8자 트랙(`docs/03_track.md`) 위에 올리면 3초 뒤 자동 출발
- [ ] **[확인]** 라인을 따라감. 안 되면 → LINE_ON 극성 / 센서 높이 / 속도(BASE_SPEED↓) / 좌우 센서 배선
- [ ] 잘 되면 확정한 속도값도 `robot.ino` 에 반영

## STEP 5 — 무선 링크 단독 검증 (보드 2개)
- [ ] `04_nrf_ping` 을 허브측 보드에 `#define NODE 0`, 로봇측 보드에 `#define NODE 1` 로 업로드
- [ ] **[확인]** 허브측 시리얼에 `[hub] ack OK ...` → 무선 정상
- [ ] `NO-ACK` 면: 어댑터 **VCC 5V**, 주소 RBT01 일치, CE9/CSN10/SPI, 모듈 교체

## STEP 6 — 본 펌웨어 무선 1:1 (허브 ↔ 로봇1)
- [ ] 로봇1에 `firmware/robot/robot.ino` 업로드 (ROBOT_ID=1, 캘리브레이션 값 반영본)
- [ ] 허브 UNO 에 `firmware/hub/hub.ino` 업로드, Mac 에 USB 연결
- [ ] 포트 확인: `ls /dev/tty.*` (usbserial/usbmodem)
- [ ] `python server/fleet_server.py --port /dev/tty.usbserial-XXXX`
- [ ] **[확인]** 서버에서 `status` → 로봇1 보임 / `go 1` 출발 / `halt 1` 정지

## STEP 7 — 3대로 확장
- [ ] 로봇2(ROBOT_ID=2), 로봇3(ROBOT_ID=3) 업로드 + 배선 (각 1대씩 STEP 3~4 약식 재검증 권장)
- [ ] 서버 `status` 에 3대 모두 보이는지
- [ ] `run` → 3대 동시 출발
- [ ] **[확인]** 8자 교차로에서 **한 대씩만 통과** + 로그:
      `[교통] 교차로 점유 → ...` / `... 통과 완료 → 해제`

### 교차로 구역락은 이렇게 구현돼 있다 (별도 코딩 불필요 — 이미 구현됨)
> 핵심 발상: **교차로 = 공유 자원, 한 번에 한 대만 = 상호배제(mutex).** 백엔드의 분산 락/임계구역과 동일.
> 차량 펌웨어(`robot.ino`)와 관제(`server/fleet_server.py`)에 이미 들어 있다. 흐름:

1. **차량**: 두 IR 센서가 동시에 검정(가로선) → 교차로로 판단, `ST_WAIT_NODE` 로 **정지**하고 상태를 무선 보고. (robot.ino line 129~131)
2. **관제 `_traffic()`**: 그 차량이 `WAIT_NODE` 인데 `intersection_owner` 가 비어 있으면 → **점유권 부여**(owner=그 차량), 그 차량에만 `GO_THROUGH` 명령 송신, `[교통] 교차로 점유` 로그. (fleet_server.py line 67~76)
3. **이미 점유 중이면**: 뒤따른 차량은 명령을 못 받아 `WAIT_NODE` 에서 그대로 **대기** → 충돌·교착 없음.
4. **차량**: `GO_THROUGH` 받으면 350ms 직진으로 교차로를 빠져나간 뒤 `ST_RUNNING` 복귀. (robot.ino line 142~147)
5. **관제**: 그 차량이 다시 `RUNNING` 으로 보고되면 → **점유권 해제**(owner=None), `RUN` 으로 되돌림, `[교통] 통과 완료 → 해제` 로그. (fleet_server.py line 78~85)

> 즉 "구현"은 끝나 있고, STEP 7에서 하는 건 **그 로직이 실물에서 의도대로 도는지 검증**하는 것.
> (이 mutex 서사가 자소서·면접의 핵심 무기 → `~/Downloads/nix/AGV_면접카드.md` §4·5)
> 트랙은 교차로 1개인 **8자**부터(관제 코드도 교차로 1개 가정). 격자(田) 다중 교차로는 다중 자원 락이라 확장 과제.

## STEP 8 — LLM 두뇌 연결
- [ ] fleet_server 실행 중 상태로, 새 터미널: `python brain/llm_orchestrator.py`
- [ ] **[확인]** "전부 출발" / "2번 멈춰" / "상태 알려줘" 자연어로 지휘됨

---

## 통합 트러블슈팅
| 증상 | 어디부터 | 원인/해결 |
|------|----------|-----------|
| 바퀴 방향 반대 | STEP 3-① | IN1↔IN2 (왼) / IN3↔IN4 (오) 스왑 |
| 라인 못 따라감 | STEP 4 | LINE_ON 극성, 센서 높이, 바닥 대비, BASE_SPEED↓ |
| 좌우 반대로 꺾음 | STEP 4 | 모터 방향 스왑 or 좌우 센서 배선 바뀜 |
| 교차로 안 멈춤 | STEP 7 | 가로선 굵기, 두 센서 동시 감지 여부 |
| 무선 no-ack | STEP 5 | 어댑터 VCC 5V, 주소 RBT0x 일치, 모듈 교체 |
| 한 대만 안 움직임 | STEP 7 | 그 로봇 ROBOT_ID 중복 업로드? 배선 |
| 포트 못 찾음 | STEP 6 | `ls /dev/tty.*` 에서 usbserial/usbmodem 확인 |

## 한눈 흐름
```
개봉 → 1대 조립 → [진단3종으로 값 확정] → robot.ino 반영
   → 단독 라인추종(05_solo) → 무선핑(04_nrf) → robot.ino 무선 1:1
   → 3대 확장(교차로 구역락) → fleet_server → LLM 지휘
```
