#include <AFMotor.h>

AF_DCMotor motorL(1);  // M1 왼쪽
AF_DCMotor motorR(4);  // M4 오른쪽

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== HW-130 모터 테스트 ==="));
}

void loop() {
  Serial.println(F("왼쪽 전진"));
  motorL.setSpeed(200); motorL.run(FORWARD);
  delay(2000); motorL.run(RELEASE);  delay(500);

  Serial.println(F("오른쪽 전진"));
  motorR.setSpeed(200); motorR.run(FORWARD);
  delay(2000); motorR.run(RELEASE);  delay(500);

  Serial.println(F("양쪽 전진"));
  motorL.setSpeed(200); motorL.run(FORWARD);
  motorR.setSpeed(200); motorR.run(FORWARD);
  delay(2000);
  motorL.run(RELEASE); motorR.run(RELEASE); delay(1000);
}
