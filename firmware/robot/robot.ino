/*
 * AGV Fleet — Robot Firmware
 * 보드 : Arduino UNO + HW-130 모터쉴드 + nRF24L01 (소프트SPI, 아날로그 포트)
 *
 * 배선:
 *   CE→5V직결(더미D6), CSN→A1, SCK→A2, MO→A3, MI→A4
 *   IR_LEFT→A0, IR_RIGHT→A5
 *
 * 라이브러리: AFMotor, RF24 (RF24_config.h 에 SOFTSPI 활성화)
 *
 * 업로드 전 ROBOT_ID 를 1/2/3 으로 변경
 */

#include <AFMotor.h>
#define SOFTSPI
#define SOFT_SPI_MISO_PIN A4
#define SOFT_SPI_MOSI_PIN A3
#define SOFT_SPI_SCK_PIN  A2
#include <SPI.h>
#include <RF24.h>

// ─── 설정 ────────────────────────────────────────
#define ROBOT_ID   1

#define IR_LEFT    A0
#define IR_RIGHT   A5
#define LINE_ON    LOW

#define BASE_SPEED  130
#define SLOW_SPEED  20
#define TURN_SPEED  160
#define TRIM_R      30   // 오른쪽 모터 보정값 (오른쪽 약하면 +, 강하면 -)

#define NUDGE_MS   150   // 교차로 중심까지 전진 시간 (튜닝)
#define TURN_MS    650   // 90° 제자리 선회 시간      (튜닝)

#define PIN_CE  6        // 더미 — CE 5V 직결
#define PIN_CSN A1

// ─── 모터 ────────────────────────────────────────
AF_DCMotor motorL(1);   // M1 왼쪽
AF_DCMotor motorR(4);   // M4 오른쪽

// ─── 무선 ────────────────────────────────────────
RF24 radio(PIN_CE, PIN_CSN);
const uint8_t robotAddr[3][6] = {"RBT01", "RBT02", "RBT03"};

struct Command { uint8_t mode; uint8_t speed; };
struct Status  { uint8_t robot_id; uint8_t state; uint8_t node; uint8_t obstacle; };

enum CmdMode   { MODE_STOP=0, MODE_RUN=1, MODE_STRAIGHT=2, MODE_LEFT=3, MODE_RIGHT=4 };
enum RobotSt   { ST_IDLE=0, ST_RUNNING=1, ST_WAIT_NODE=2, ST_NUDGE=3, ST_TURNING=4 };

Command cmd = {MODE_STOP, BASE_SPEED};
Status  st  = {ROBOT_ID, ST_IDLE, 0, 0};

uint8_t  pendingTurn  = MODE_STRAIGHT;
uint32_t stateStart   = 0;
uint32_t lastNode     = 0;
uint32_t bothOnStart  = 0;    // L&&R 최초 감지 시각 (교차로 확인용)
#define  INTERSECT_MS  50     // 교차로 판정: 50ms 이상 지속돼야 인정

// ─── 모터 헬퍼 ───────────────────────────────────
// 왼쪽 모터 배선 반대 → FORWARD/BACKWARD 뒤집음
#define L_FWD BACKWARD
#define L_BWD FORWARD

void stopMotors() {
  motorL.run(RELEASE); motorR.run(RELEASE);
}
void goForward(uint8_t s) {
  uint8_t sr = (s + TRIM_R > 255) ? 255 : s + TRIM_R;
  motorL.setSpeed(s);  motorL.run(L_FWD);
  motorR.setSpeed(sr); motorR.run(FORWARD);
}
void pivotLeft(uint8_t s) {
  motorL.setSpeed(s); motorL.run(L_BWD);
  motorR.setSpeed(s); motorR.run(FORWARD);
}
void pivotRight(uint8_t s) {
  motorL.setSpeed(s); motorL.run(L_FWD);
  motorR.setSpeed(s); motorR.run(BACKWARD);
}
void lineFollow(bool L, bool R) {
  uint8_t s  = cmd.speed ? cmd.speed : BASE_SPEED;
  uint8_t sr = (s + TRIM_R > 255) ? 255 : s + TRIM_R;
  if (!L && !R) {
    motorL.setSpeed(s);  motorL.run(L_FWD);
    motorR.setSpeed(sr); motorR.run(FORWARD);
  } else if (L && !R) {
    motorL.setSpeed(s);          motorL.run(L_FWD);
    motorR.setSpeed(SLOW_SPEED); motorR.run(FORWARD);
  } else if (!L && R) {
    motorL.setSpeed(SLOW_SPEED); motorL.run(L_FWD);
    motorR.setSpeed(sr);         motorR.run(FORWARD);
  }
}

void enterState(uint8_t s) { st.state = s; stateStart = millis(); }

// ─── setup ───────────────────────────────────────
void setup() {
  Serial.begin(115200);
  motorL.run(RELEASE); motorR.run(RELEASE);
  pinMode(IR_LEFT, INPUT); pinMode(IR_RIGHT, INPUT);

  if (!radio.begin()) {
    Serial.println(F("nRF24 init fail")); while (1);
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  radio.openReadingPipe(1, robotAddr[ROBOT_ID - 1]);
  radio.startListening();
  radio.writeAckPayload(1, &st, sizeof(st));

  Serial.print(F("[robot")); Serial.print(ROBOT_ID); Serial.println(F("] ready"));
}

// ─── loop ────────────────────────────────────────
void loop() {
  // 1) 명령 수신
  if (radio.available()) radio.read(&cmd, sizeof(cmd));

  // 2) IR
  bool L = (digitalRead(IR_LEFT)  == LINE_ON);
  bool R = (digitalRead(IR_RIGHT) == LINE_ON);
  uint32_t now = millis();

  // 3) 상태 머신
  if (cmd.mode == MODE_STOP) {
    stopMotors(); enterState(ST_IDLE);

  } else {
    switch (st.state) {

      case ST_IDLE:
        if (cmd.mode == MODE_RUN) enterState(ST_RUNNING);
        break;

      case ST_RUNNING:
        if (L && R) {
          if (bothOnStart == 0) bothOnStart = now;
          if (now - bothOnStart >= INTERSECT_MS && now - lastNode > 600) {
            bothOnStart = 0;
            lastNode = now;
            st.node++;
            stopMotors();
            enterState(ST_WAIT_NODE);
            Serial.print(F("node ")); Serial.println(st.node);
          }
        } else {
          bothOnStart = 0;
          lineFollow(L, R);
        }
        break;

      case ST_WAIT_NODE:                        // 명령 대기
        if (cmd.mode >= MODE_STRAIGHT) {
          pendingTurn = cmd.mode;
          goForward(BASE_SPEED);
          enterState(ST_NUDGE);
        } else if (cmd.mode == MODE_RUN) {     // RUN 명령으로도 재개 가능
          enterState(ST_RUNNING);
        }
        break;

      case ST_NUDGE:                            // 교차로 중심 통과
        if (now - stateStart >= NUDGE_MS) {
          if (pendingTurn == MODE_STRAIGHT) {
            enterState(ST_RUNNING);
          } else if (pendingTurn == MODE_LEFT) {
            pivotLeft(TURN_SPEED);
            enterState(ST_TURNING);
          } else {
            pivotRight(TURN_SPEED);
            enterState(ST_TURNING);
          }
        }
        break;

      case ST_TURNING:                          // 선회 완료 → 라인추종
        if (now - stateStart >= TURN_MS) {
          stopMotors();
          enterState(ST_RUNNING);
        }
        break;
    }
  }

  // 4) ACK 페이로드 갱신 (허브 폴링 시 전달)
  st.robot_id = ROBOT_ID;
  radio.writeAckPayload(1, &st, sizeof(st));
}
