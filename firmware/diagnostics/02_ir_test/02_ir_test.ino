/*
 * AGV 진단 ② IR 라인센서 테스트
 * 목적 : 검정 라인 위 / 흰 바닥 위에서 두 센서의 디지털 출력을 읽어
 *        (1) LINE_ON 극성, (2) 센서 높이, (3) 두 센서 동시감지(교차로) 가능 여부 확인.
 *
 * 사용법
 *   1) USB 연결, 시리얼 모니터 115200 baud
 *   2) 센서를 흰 바닥 → 검정 테이프 위로 천천히 옮기며 값 변화를 본다
 *   3) 키트에 가변저항(감도조절)이 있으면 검정/흰 구분이 또렷해지게 조절
 *
 * 기대 출력 (200ms 주기):  L=<0/1>  R=<0/1>
 *
 * 판정/기록 → robot.ino 의 LINE_ON (line 36) 결정
 *   - 검정 라인 위에서 값이 1(HIGH) 이면  → #define LINE_ON HIGH   (robot.ino 기본값)
 *   - 검정 라인 위에서 값이 0(LOW)  이면  → #define LINE_ON LOW
 *   ※ robot.ino 코드는 "L,R 이 LINE_ON 과 같으면 검정 위"로 해석한다.
 *
 * 높이/감도 점검
 *   - 흰↔검정 전환이 또렷하지 않으면 센서를 바닥에 더 가깝게(보통 5~15mm)
 *   - 두 센서가 가로선 위에서 거의 동시에 1이 되어야 교차로(양쪽 검정) 감지가 됨
 *
 * ※ 키트가 아날로그 출력 센서면 아래 USE_ANALOG 를 1 로 바꿔 raw 값도 같이 본다.
 */

#define IR_LEFT   A0   // robot_grid.ino 와 동일
#define IR_RIGHT  A5   // (A1 은 nRF24 CSN — 오른쪽 IR 은 A5)

#define USE_ANALOG 1   // 1로 바꾸면 analogRead raw 값도 출력(아날로그 센서일 때 임계값 잡기용)

void setup() {
  Serial.begin(115200);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  Serial.println(F("=== IR sensor test ===  검정=라인, 흰=바닥. L/R 값 관찰"));
}

void loop() {
  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  Serial.print(F("L="));  Serial.print(L);
  Serial.print(F("  R=")); Serial.print(R);

#if USE_ANALOG
  Serial.print(F("   (raw L="));
  Serial.print(analogRead(IR_LEFT));
  Serial.print(F(" R="));
  Serial.print(analogRead(IR_RIGHT));
  Serial.print(F(")"));
#endif

  if (L && R)        Serial.println(F("   -> 양쪽 검정 = 교차로"));
  else if (L && !R)  Serial.println(F("   -> 왼쪽만 검정"));
  else if (!L && R)  Serial.println(F("   -> 오른쪽만 검정"));
  else               Serial.println(F("   -> 둘 다 흰 = 라인이 가운데"));

  delay(200);
}
