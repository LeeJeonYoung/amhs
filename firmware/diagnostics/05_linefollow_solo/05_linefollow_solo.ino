#include <AFMotor.h>

AF_DCMotor motorL(1);  // M1 왼쪽
AF_DCMotor motorR(4);  // M4 오른쪽

#define IR_LEFT   A0
#define IR_RIGHT  A5
#define LINE_ON   HIGH      // 검정 감지 = HIGH (키트 기준)

#define BASE_SPEED 80
#define SLOW_SPEED 30

void lineFollow(bool L, bool R) {
  if (!L && !R) {
    // 흰 바닥 — 직진
    motorL.setSpeed(BASE_SPEED); motorL.run(FORWARD);
    motorR.setSpeed(BASE_SPEED); motorR.run(FORWARD);
  } else if (L && !R) {
    // 왼쪽 라인 감지 → 오른쪽으로 부드럽게 커브
    motorL.setSpeed(BASE_SPEED); motorL.run(FORWARD);
    motorR.setSpeed(SLOW_SPEED); motorR.run(FORWARD);
  } else if (!L && R) {
    // 오른쪽 라인 감지 → 왼쪽으로 부드럽게 커브
    motorL.setSpeed(SLOW_SPEED); motorL.run(FORWARD);
    motorR.setSpeed(BASE_SPEED); motorR.run(FORWARD);
  } else {
    // 양쪽 검정(교차로) — 직진 통과
    motorL.setSpeed(BASE_SPEED); motorL.run(FORWARD);
    motorR.setSpeed(BASE_SPEED); motorR.run(FORWARD);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  motorL.run(RELEASE);
  motorR.run(RELEASE);
  Serial.println(F("=== solo line-follow === 3초 뒤 출발"));
  delay(3000);
}

void loop() {
  bool L = (digitalRead(IR_LEFT)  == LINE_ON);
  bool R = (digitalRead(IR_RIGHT) == LINE_ON);
  lineFollow(L, R);
}
