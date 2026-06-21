# 05. 캘리브레이션 — 진단 스케치로 부위별 검증 후 robot.ino 반영

> 부품 도착 직후, 본 펌웨어(`robot.ino`)를 올리기 **전에** 부위별로 따로 검증한다.
> 각 스케치에서 얻은 값을 `robot.ino` 에 그대로 옮기면 1대 검증이 훨씬 빨라진다.
> 스케치 위치: `firmware/diagnostics/0{1..5}_*/` (각 .ino 는 동명 폴더 안 — 아두이노 규칙)

## 순서 (위 → 아래)

| 순서 | 스케치 | 확인할 것 | 기록할 값 |
|------|--------|-----------|-----------|
| 1 | `01_motor_test` | 좌/우 모터 방향(전진=정방향?) | 모터 방향 정상 / 스왑 필요 여부 |
| 2 | `02_ir_test` | 검정 라인 위 센서 출력값 | **LINE_ON** = HIGH or LOW |
| 3 | `03_sonar_test` | 거리 측정 정상·정지 거리 | **OBSTACLE_CM** (cm) |
| 4 | `05_linefollow_solo` | 1대 라인추종(무선 없이) | BASE_SPEED/TURN_SPEED 적정값 |
| 5 | `04_nrf_ping` | 무선 링크(보드 2개) | ack OK / no-ack |

> 1~4는 로봇 1대로, 5는 보드 2개(허브측+로봇측)로. 5는 무선 1:1 단계 직전에 해도 된다.

---

## 1) 모터 — `01_motor_test`
- 바퀴를 **공중에 띄운 채** 업로드, 시리얼 모니터 115200.
- 출력 라벨(`LEFT forward` 등)과 실제 회전 방향 대조.
- **반대로 돌면** → `robot.ino` 에서 IN 핀 스왑:
  - 왼쪽 반대 : `IN1`(2) ↔ `IN2`(4) — robot.ino line 27~28
  - 오른쪽 반대 : `IN3`(7) ↔ `IN4`(8) — robot.ino line 29~30
- 한쪽이 안 돌면 ENA/ENB(PWM 5/6) 배선·모터선 점검.

## 2) IR 센서 — `02_ir_test`
- 센서를 흰 바닥 → 검정 테이프 위로 옮기며 `L=/R=` 값 관찰.
- **검정 라인 위에서 1(HIGH)** → `#define LINE_ON HIGH` (robot.ino **line 36**, 기본값)
- **검정 라인 위에서 0(LOW)** → `#define LINE_ON LOW`
- 흰↔검정 전환이 또렷하지 않으면: 센서 높이 5~15mm로, 키트 감도 가변저항 조절, 바닥은 밝게.
- 가로선 위에서 **두 센서가 거의 동시에 1**이 되어야 교차로 감지 OK.

## 3) 초음파 — `03_sonar_test`
- 손/벽을 가까이↔멀리. `dist = NN cm` 가 안정적으로 변하면 정상.
- 멈추고 싶은 거리를 `OBSTACLE_CM` 에 기록 — robot.ino **line 41** (기본 12).
- 계속 `999` 만 나오면 TRIG(A2)/ECHO(A3) 배선·전원 점검.

## 4) 단독 라인추종 — `05_linefollow_solo`
- 위 1~3 확정 후 업로드. 8자 트랙에 올리면 3초 뒤 자동 출발.
- 라인을 잘 따라가도록 속도 조정:
  - 너무 빨라 라인 이탈 → `BASE_SPEED` ↓ (robot.ino **line 44**)
  - 회전이 굼떠 못 꺾음 → `TURN_SPEED` ↑ (robot.ino **line 45**)
- 좌우 반대로 꺾으면: 모터 방향(IN 스왑) 또는 좌우 센서 배선 바뀜.
- 교차로를 그냥 통과하는 건 **정상**(단독 모드는 관제가 없음).

## 5) 무선 핑 — `04_nrf_ping` (보드 2개)
- 허브측 보드: `#define NODE 0` 으로 업로드 / 로봇측 보드: `#define NODE 1`.
- 둘 다 nRF24만 꽂혀 있으면 됨. 허브측 시리얼에 `ack OK` 가 뜨면 무선 정상.
- `NO-ACK` 면: 어댑터 **VCC 를 5V** 에 꽂았는지, 주소(RBT01) 일치, CE9/CSN10/SPI 배선, 모듈 교체.

---

## robot.ino 반영 요약 (확정값 옮기는 곳)

| 값 | robot.ino 위치 | 출처 스케치 |
|----|----------------|-------------|
| `LINE_ON` (HIGH/LOW) | line 36 | ② IR |
| `OBSTACLE_CM` | line 41 | ③ 초음파 |
| `BASE_SPEED` | line 44 | ⑤ 단독주행 |
| `TURN_SPEED` | line 45 | ⑤ 단독주행 |
| IN1/IN2 스왑(왼쪽) | line 27~28 | ① 모터 |
| IN3/IN4 스왑(오른쪽) | line 29~30 | ① 모터 |

> 반영 후 `robot.ino`(ROBOT_ID=1)를 올려 `docs/04_run_order.md` STEP 2(무선 1:1)로 진행.
