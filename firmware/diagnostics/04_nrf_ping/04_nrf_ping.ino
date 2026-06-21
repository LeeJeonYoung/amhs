/*
 * AGV 진단 ④ nRF24L01 무선 핑 테스트
 *
 * NODE 0 = 허브 (하드웨어 SPI, 쉴드 없음)
 *   CE→D9, CSN→D10, SCK→D13, MO→D11, MI→D12, VCC→5V, GND→GND
 *
 * NODE 1 = 로봇 (소프트웨어 SPI, 쉴드 아날로그 포트)
 *   CE→5V직결, CSN→A1, SCK→A2, MO→A3, MI→A4, VCC→5V, GND→GND
 */

#define NODE 1   // 0=허브, 1=로봇

// 소프트웨어 SPI — 허브/로봇 공통: CSN=A1, SCK=A2, MO=A3, MI=A4
#define SOFTSPI
#define SOFT_SPI_MISO_PIN A4
#define SOFT_SPI_MOSI_PIN A3
#define SOFT_SPI_SCK_PIN  A2
#include <SPI.h>
#include <RF24.h>
#if NODE == 0
#define PIN_CE  9    // 허브: CE → D9 (TX 펄스 필요)
#else
#define PIN_CE  6    // 로봇: 더미 — CE 5V 직결 (항상 RX)
#endif
#define PIN_CSN A1
// ───────────────────────────────────────────────────────────────

RF24 radio(PIN_CE, PIN_CSN);
const uint8_t addr[6] = "RBT01";

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println(F("nRF24 초기화 실패 — 배선 확인"));
    while (1);
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();

#if NODE == 1
  radio.openReadingPipe(1, addr);
  radio.startListening();
  Serial.println(F("[robot] listening RBT01"));
#else
  radio.openWritingPipe(addr);
  radio.stopListening();
  Serial.println(F("[hub] sending to RBT01"));
#endif
}

#if NODE == 1
void loop() {
  if (radio.available()) {
    uint8_t cnt = 0;
    radio.read(&cnt, sizeof(cnt));
    uint8_t reply = cnt + 1;
    radio.writeAckPayload(1, &reply, sizeof(reply));
    Serial.print(F("[robot] got ")); Serial.print(cnt);
    Serial.print(F(" -> ack ")); Serial.println(reply);
  }
}
#else
uint8_t cnt = 0;
void loop() {
  bool ok = radio.write(&cnt, sizeof(cnt));
  if (ok && radio.isAckPayloadAvailable()) {
    uint8_t back = 0;
    radio.read(&back, sizeof(back));
    Serial.print(F("[hub] ack OK  sent=")); Serial.print(cnt);
    Serial.print(F(" back=")); Serial.println(back);
  } else {
    Serial.println(F("[hub] NO-ACK"));
  }
  cnt++;
  delay(500);
}
#endif
