/*
 * AGV Fleet — Hub Firmware
 * 보드 : Arduino UNO (쉴드 없음) + nRF24L01 (소프트SPI, 아날로그 포트)
 *
 * 배선: CE→D9, CSN→A1, SCK→A2, MO→A3, MI→A4
 *
 * 시리얼 프로토콜 (115200):
 *   PC→허브  : C <id> <mode> <speed>\n   예) "C 1 1 80"  (1번 RUN)
 *              mode: 0=STOP 1=RUN 2=STRAIGHT 3=LEFT 4=RIGHT
 *   허브→PC  : S <id> <state> <node>\n
 *              state: 0=IDLE 1=RUNNING 2=WAIT_NODE 3=NUDGE 4=TURNING
 *   허브→PC  : # 로그
 */

#define SOFTSPI
#define SOFT_SPI_MISO_PIN A4
#define SOFT_SPI_MOSI_PIN A3
#define SOFT_SPI_SCK_PIN  A2
#include <SPI.h>
#include <RF24.h>

#define PIN_CE  9
#define PIN_CSN A1

RF24 radio(PIN_CE, PIN_CSN);
const uint8_t robotAddr[3][6] = {"RBT01", "RBT02", "RBT03"};

struct Command { uint8_t mode; uint8_t speed; };
struct Status  { uint8_t robot_id; uint8_t state; uint8_t node; uint8_t obstacle; };

enum CmdMode { MODE_STOP=0, MODE_RUN=1, MODE_STRAIGHT=2, MODE_LEFT=3, MODE_RIGHT=4 };
enum RobotSt { ST_IDLE=0, ST_RUNNING=1, ST_WAIT_NODE=2, ST_NUDGE=3, ST_TURNING=4 };

const int N = 3;
Command  cmd[N];
uint8_t  prevState[N];
uint8_t  noAckCount[N];
String   buf;

void parse(String s) {
  s.trim();
  if (!s.length() || s[0] != 'C') return;
  int id, mode, sp;
  if (sscanf(s.c_str(), "C %d %d %d", &id, &mode, &sp) == 3
      && id >= 1 && id <= N) {
    cmd[id-1].mode  = (uint8_t)mode;
    cmd[id-1].speed = (uint8_t)sp;
    Serial.print(F("# cmd robot ")); Serial.print(id);
    Serial.print(F(" mode=")); Serial.println(mode);
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { parse(buf); buf = ""; }
    else if (c != '\r') buf += c;
  }
}

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println(F("# nRF24 init fail")); while (1);
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  for (int i = 0; i < N; i++) {
    cmd[i].mode = MODE_STOP; cmd[i].speed = 80;
    prevState[i] = ST_IDLE;
    noAckCount[i] = 0;
  }
  Serial.println(F("# hub ready"));
  Serial.println(F("# C <id> <mode> <speed>  예) C 1 1 80"));
}

void loop() {
  handleSerial();

  for (int i = 0; i < N; i++) {
    radio.stopListening();
    radio.openWritingPipe(robotAddr[i]);
    bool ok = radio.write(&cmd[i], sizeof(Command));

    if (ok && radio.isAckPayloadAvailable()) {
      Status stt;
      radio.read(&stt, sizeof(stt));
      radio.flush_rx();  // FIFO 잔여 데이터 제거 (노이즈 방지)

      // robot_id 검증 — 엉뚱한 값이면 무시
      if (stt.robot_id != (uint8_t)(i + 1)) {
        noAckCount[i]++;
      } else {
        noAckCount[i] = 0;

        // 교차로 도착 알림
        if (stt.state == ST_WAIT_NODE && prevState[i] != ST_WAIT_NODE) {
          Serial.print(F("# robot ")); Serial.print(i+1);
          Serial.print(F(" at node ")); Serial.print(stt.node);
          Serial.println(F(" — C <id> 2/3/4 <speed> 로 방향 지정"));
        }

        // 선회 완료 후 명령 자동 리셋
        if (stt.state == ST_RUNNING && prevState[i] == ST_TURNING
            && cmd[i].mode >= MODE_STRAIGHT) {
          cmd[i].mode = MODE_RUN;
        }

        prevState[i] = stt.state;

        Serial.print(F("S ")); Serial.print(stt.robot_id);
        Serial.print(' ');     Serial.print(stt.state);
        Serial.print(' ');     Serial.println(stt.node);
      }
    } else {
      radio.flush_rx();
      noAckCount[i]++;
      if (noAckCount[i] == 20) {  // ~160ms 연속 무응답 → 경고 1회
        Serial.print(F("# robot ")); Serial.print(i+1);
        Serial.println(F(" NO-ACK — 배터리/배선 확인"));
      }
    }
    delay(8);
  }
}
