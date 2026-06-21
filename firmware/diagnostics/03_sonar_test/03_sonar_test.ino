/*
 * AGV 진단 ③ 초음파(HC-SR04) 테스트
 * 목적 : 거리 측정이 정상인지 확인하고 OBSTACLE_CM 임계값을 정한다.
 *
 * 사용법
 *   1) USB 연결, 시리얼 모니터 115200 baud
 *   2) 손/벽을 센서 앞에서 가까이↔멀리 움직이며 cm 값을 본다
 *   3) "이 거리부터 멈췄으면" 하는 지점을 robot.ino 의 OBSTACLE_CM(line 41)에 기록
 *
 * 기대 출력 (100ms 주기):  dist = <cm> cm   [측정실패면 999]
 *
 * 판정/기록
 *   - 값이 안정적으로 변하면 정상. 999 만 계속 나오면 TRIG/ECHO 배선/전원 점검
 *   - robot.ino 기본 OBSTACLE_CM = 12. 너무 예민하면 ↑, 늦게 멈추면 ↓
 *
 * ※ readDistanceCm() 로직·타임아웃(20ms)·핀(TRIG=A2, ECHO=A3) 은 robot.ino 와 동일.
 */

#define TRIG  A2   // robot.ino 와 동일
#define ECHO  A3

long readDistanceCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10); digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 20000UL); // 20ms 타임아웃
  if (dur == 0) return 999;                // 측정 실패/범위 밖
  return dur / 58;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.println(F("=== sonar test ===  앞에서 손을 가까이/멀리 해보세요"));
}

void loop() {
  long d = readDistanceCm();
  Serial.print(F("dist = "));
  Serial.print(d);
  Serial.println(F(" cm"));
  delay(100);
}
