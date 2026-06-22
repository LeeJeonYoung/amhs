/*
 * AGV Fleet — Robot Firmware (격자 버전)
 * ─────────────────────────────────────────────────────────────
 * robot.ino(8자 검증용)는 그대로 보존하고, 격자(4x4 등)용으로 분리한 펌웨어.
 * 차이점: 교차로에서 직진뿐 아니라 좌/우/유턴까지 4방향을 받는다.
 *
 * 명령 프로토콜(서버 navigator.py 와 동일):
 *   0 STOP / 1 RUN(라인추종 재개) / 2 STRAIGHT / 3 RIGHT / 4 UTURN / 5 LEFT
 *
 * 보드 : Arduino UNO + HW-130 모터쉴드 + nRF24L01 (소프트SPI, 아날로그 포트)
 * 배선 : CE→5V직결(더미D6), CSN→A1, SCK→A2, MO→A3, MI→A4 / IR_LEFT→A0, IR_RIGHT→A5
 * 라이브러리 : AFMotor, RF24 (RF24_config.h 에 SOFTSPI 활성화)
 * 업로드 전 ROBOT_ID 를 1/2/3 으로 변경.
 *
 * ※ 주행 튜닝값(LINE_ON, BASE_SPEED, TRIM_R, L_FWD …)은 robot.ino 실주행 확정값을 그대로 가져옴.
 */
#include <AFMotor.h>
#define SOFTSPI
#define SOFT_SPI_MISO_PIN A4
#define SOFT_SPI_MOSI_PIN A3
#define SOFT_SPI_SCK_PIN  A2
#include <SPI.h>
#include <RF24.h>

// ─── 설정 ────────────────────────────────────────
// 업로드 시 덮어쓰기 가능: arduino-cli ... --build-property build.extra_flags=-DROBOT_ID=2
#ifndef ROBOT_ID
#define ROBOT_ID   1
#endif

#define IR_LEFT    A0
#define IR_RIGHT   A5
#ifndef LINE_ON
#define LINE_ON    LOW          // 검정=LOW, 흰=HIGH (기본). 센서별로 반대면 -DLINE_ON=HIGH 로 빌드
#endif

#define BASE_SPEED  130
#define SLOW_SPEED  20
#define TURN_SPEED  230         // 제자리 피벗 토크(약한 배터리/마찰 대응 — 안 돌면 더 올림)
#define TRIM_R      30          // 오른쪽 모터 보정(+: 오른쪽이 약함)

#define NUDGE_MS    150         // 교차로 중심까지 전진 시간
#define TURN_MS     650         // 90° 제자리 선회 시간
#define UTURN_MS    1300        // 180° 선회 = 90°의 약 2배(실측 후 조정)
#define INTERSECT_MS 50         // 교차로 판정: 50ms 이상 지속돼야 인정(오인식 방지)
#define RENODE_MS   300         // 같은 교차로 재검출 방지(통과 직후 무시 구간) — 회전 직후 가까운 교차로도 잡게 단축
#define KICK_MS     300         // 출발/회전 직후 정지마찰 극복 부스트 시간
#define KICK_SPEED  255         // 부스트 속도(최대) — 첫 KICK_MS 동안만
#define LOST_MS     4000        // 라인 접촉(센서/교차로) 없이 이 시간 넘으면 = 길잃음/벽
#define RECOVER_MS  900         // 복구 후진 최대 시간(라인 재접촉 시 즉시 정지)

#define PIN_CE  6               // 더미 — CE 5V 직결
#define PIN_CSN A1

// ─── 모터 ────────────────────────────────────────
AF_DCMotor motorL(1);          // M1 왼쪽
AF_DCMotor motorR(4);          // M4 오른쪽

// 왼쪽 모터 배선 반대 → FORWARD/BACKWARD 뒤집음(실주행 확정)
#define L_FWD BACKWARD
#define L_BWD FORWARD

// ─── 무선 ────────────────────────────────────────
RF24 radio(PIN_CE, PIN_CSN);
const uint8_t robotAddr[3][6] = {"RBT01", "RBT02", "RBT03"};

struct Command { uint8_t mode; uint8_t speed; };
struct Status  { uint8_t robot_id; uint8_t state; uint8_t node; uint8_t obstacle; };

enum CmdMode { MODE_STOP=0, MODE_RUN=1, MODE_STRAIGHT=2, MODE_RIGHT=3, MODE_UTURN=4, MODE_LEFT=5, MODE_FWD=6 };
enum RobotSt { ST_IDLE=0, ST_RUNNING=1, ST_WAIT_NODE=2, ST_NUDGE=3, ST_TURNING=4, ST_RECOVER=5 };

Command cmd = {MODE_STOP, BASE_SPEED};
Status  st  = {ROBOT_ID, ST_IDLE, 0, 0};

uint8_t  pendingTurn = MODE_STRAIGHT;
uint32_t turnDur     = 0;       // 이번 회전에 쓸 시간(90°/180°)
uint32_t stateStart  = 0;
uint32_t lastNode    = 0;
uint32_t bothOnStart = 0;       // L&&R 최초 감지 시각
uint32_t lastLineEvent = 0;     // 마지막 라인 접촉(센서/교차로) 시각 — 길잃음 판정용

// ─── 모터 헬퍼 ───────────────────────────────────
void stopMotors() { motorL.run(RELEASE); motorR.run(RELEASE); }

void goForward(uint8_t s) {
  uint8_t sr = (s + TRIM_R > 255) ? 255 : s + TRIM_R;
  motorL.setSpeed(s);  motorL.run(L_FWD);
  motorR.setSpeed(sr); motorR.run(FORWARD);
}
void goBackward(uint8_t s) {     // 복구용 후진(양쪽 뒤로)
  motorL.setSpeed(s); motorL.run(L_BWD);
  motorR.setSpeed(s); motorR.run(BACKWARD);
}
void pivotLeft(uint8_t s) {     // 전진하며 좌선회(호) — 안쪽(왼쪽) 느리게, 후진 없음
  motorL.setSpeed(SLOW_SPEED); motorL.run(L_FWD);
  motorR.setSpeed(s);          motorR.run(FORWARD);
}
void pivotRight(uint8_t s) {    // 전진하며 우선회(호) — 안쪽(오른쪽) 느리게, 후진 없음
  motorL.setSpeed(s);          motorL.run(L_FWD);
  motorR.setSpeed(SLOW_SPEED); motorR.run(FORWARD);
}
void lineFollow(bool L, bool R) {
  uint8_t cs = cmd.speed ? cmd.speed : BASE_SPEED;
  uint8_t s  = (millis() - stateStart < KICK_MS && cs < KICK_SPEED) ? KICK_SPEED : cs;  // 출발 킥스타트(정지마찰 극복)
  uint8_t sr = (s + TRIM_R > 255) ? 255 : s + TRIM_R;
  if (!L && !R) {                       // 직진
    motorL.setSpeed(s);  motorL.run(L_FWD);
    motorR.setSpeed(sr); motorR.run(FORWARD);
  } else if (L && !R) {                 // 왼쪽 센서 검정 → 오른쪽으로 보정 (조향 반전 수정)
    motorL.setSpeed(SLOW_SPEED); motorL.run(L_FWD);
    motorR.setSpeed(sr);         motorR.run(FORWARD);
  } else if (!L && R) {                 // 오른쪽 센서 검정 → 왼쪽으로 보정 (조향 반전 수정)
    motorL.setSpeed(s);          motorL.run(L_FWD);
    motorR.setSpeed(SLOW_SPEED); motorR.run(FORWARD);
  }
}

void enterState(uint8_t s) { st.state = s; stateStart = millis(); if (s == ST_RUNNING) lastLineEvent = millis(); }

// ─── setup ───────────────────────────────────────
void setup() {
  Serial.begin(115200);
  motorL.run(RELEASE); motorR.run(RELEASE);
  pinMode(IR_LEFT, INPUT); pinMode(IR_RIGHT, INPUT);

  if (!radio.begin()) { Serial.println(F("nRF24 init fail")); while (1); }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  radio.openReadingPipe(1, robotAddr[ROBOT_ID - 1]);
  radio.startListening();
  radio.writeAckPayload(1, &st, sizeof(st));

  Serial.print(F("[robot")); Serial.print(ROBOT_ID); Serial.println(F("] grid ready"));
}

// ─── loop ────────────────────────────────────────
void loop() {
  if (radio.available()) radio.read(&cmd, sizeof(cmd));

  bool L = (digitalRead(IR_LEFT)  == LINE_ON);
  bool R = (digitalRead(IR_RIGHT) == LINE_ON);
  uint32_t now = millis();

  if (cmd.mode == MODE_STOP) {
    stopMotors(); enterState(ST_IDLE);

  } else if (cmd.mode == MODE_FWD) {                  // 강제 직진: IR·상태기계 무시하고 전진(모터 점검·데모용)
    goForward(cmd.speed ? cmd.speed : BASE_SPEED);
    st.state = ST_RUNNING;

  } else {
    switch (st.state) {

      case ST_IDLE:
        if (cmd.mode == MODE_RUN) enterState(ST_RUNNING);
        break;

      case ST_RUNNING:
        if (L || R) lastLineEvent = now;                // 라인 접촉(센서/교차로) → 길잃음 타이머 리셋
        if (L && R) {                                   // 교차로(가로선) 감지
          if (bothOnStart == 0) bothOnStart = now;
          if (now - bothOnStart >= INTERSECT_MS && now - lastNode > RENODE_MS) {
            bothOnStart = 0;
            lastNode = now;
            st.node++;
            stopMotors();
            enterState(ST_WAIT_NODE);                   // 서버에 '도착, 명령 대기' 보고
            Serial.print(F("node ")); Serial.println(st.node);
          }
        } else {
          bothOnStart = 0;
          lineFollow(L, R);
          if (now - lastLineEvent > LOST_MS) {          // 라인 한참 못 봄 = 길잃음/벽 → 후진 복구
            goBackward(BASE_SPEED);
            enterState(ST_RECOVER);
            Serial.println(F("lost line -> recover"));
          }
        }
        break;

      case ST_WAIT_NODE:                                // 교차로 도착 → 정지하고 대기 (RUN으론 재개 안 함)
        if (cmd.mode >= MODE_STRAIGHT) {                // 직진/좌/우/유턴(2~5)을 줘야 다시 출발
          pendingTurn = cmd.mode;
          turnDur = (cmd.mode == MODE_UTURN) ? UTURN_MS : TURN_MS;
          goForward(BASE_SPEED);
          enterState(ST_NUDGE);
        }
        break;

      case ST_NUDGE:                                    // 교차로 중심 통과
        if (now - stateStart >= NUDGE_MS) {
          if (pendingTurn == MODE_STRAIGHT) {
            enterState(ST_RUNNING);
          } else if (pendingTurn == MODE_LEFT || pendingTurn == MODE_UTURN) {
            pivotLeft(TURN_SPEED);                      // 유턴은 좌선회를 길게
            enterState(ST_TURNING);
          } else {                                      // MODE_RIGHT
            pivotRight(TURN_SPEED);
            enterState(ST_TURNING);
          }
        }
        break;

      case ST_TURNING:                                  // 선회 완료 → 라인추종
        if (now - stateStart >= turnDur) {
          stopMotors();
          lastNode = now;                               // 회전 직후 같은 노드 재검출 방지
          enterState(ST_RUNNING);
        }
        break;

      case ST_RECOVER:                                  // 길잃음/벽 복구: 라인 재접촉 또는 시간초과까지 후진
        if (L || R || now - stateStart >= RECOVER_MS) {
          stopMotors();
          enterState(ST_RUNNING);                       // 라인 다시 잡고 추종 재개
        }
        break;
    }
  }

  st.robot_id = ROBOT_ID;
  radio.writeAckPayload(1, &st, sizeof(st));            // 허브 폴링 시 상태 전달
}
