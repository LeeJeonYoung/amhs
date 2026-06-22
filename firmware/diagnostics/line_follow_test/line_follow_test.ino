/* 라인추종(조향) 테스트 — 무선 없음. 켜자마자 라인추종 시작.
 * 허브·nRF24 불필요 → 배터리로 트랙 위에 놓으면 바로 라인 따라감.
 * 목적: robot_grid v6의 조향(보정) 방향이 맞는지 단독 검증.
 * 동작: 라인 따라가다 양쪽 센서 검정(교차로) 닿으면 0.5초 정지 후 계속.
 * 배선: IR 좌=A0, 우=A5 / 모터 M1=좌, M4=우, 왼쪽 반대배선(L_FWD=BACKWARD) / LINE_ON=HIGH(검정).
 */
#include <AFMotor.h>
AF_DCMotor motorL(1);   // M1 왼쪽
AF_DCMotor motorR(4);   // M4 오른쪽
#define L_FWD   BACKWARD
#define IR_LEFT  A0
#define IR_RIGHT A5
#define LINE_ON  HIGH    // 검정=HIGH (robot1/2/3 측정값)
#define BASE   140
#define SLOW    20
#define TRIM_R  30

void setup() {
  Serial.begin(115200);
  pinMode(IR_LEFT, INPUT); pinMode(IR_RIGHT, INPUT);
  motorL.run(RELEASE); motorR.run(RELEASE);
  delay(800);
  Serial.println(F("=== line follow test (무선없음, 켜자마자 추종) ==="));
}

void loop() {
  bool L = (digitalRead(IR_LEFT)  == LINE_ON);
  bool R = (digitalRead(IR_RIGHT) == LINE_ON);
  uint8_t sr = (BASE + TRIM_R > 255) ? 255 : BASE + TRIM_R;

  if (L && R) {                 // 양쪽 검정 = 교차로 → 잠깐 정지
    motorL.run(RELEASE); motorR.run(RELEASE);
    Serial.println(F("교차로 감지 → 정지"));
    delay(500);
  } else if (!L && !R) {        // 직진
    motorL.setSpeed(BASE); motorL.run(L_FWD);
    motorR.setSpeed(sr);   motorR.run(FORWARD);
  } else if (L && !R) {         // 왼쪽 센서 검정 → 오른쪽으로 보정 (v6 수정)
    motorL.setSpeed(SLOW); motorL.run(L_FWD);
    motorR.setSpeed(sr);   motorR.run(FORWARD);
  } else {                      // 오른쪽 센서 검정 → 왼쪽으로 보정 (v6 수정)
    motorL.setSpeed(BASE); motorL.run(L_FWD);
    motorR.setSpeed(SLOW); motorR.run(FORWARD);
  }
  delay(10);
}
