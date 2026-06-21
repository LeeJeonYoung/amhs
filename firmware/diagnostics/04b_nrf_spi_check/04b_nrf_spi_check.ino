/*
 * AGV 진단 ④b nRF24L01 베어메탈 SPI 배선 확인
 *
 * RF24 라이브러리 없이 직접 비트뱅잉 SPI로 STATUS 레지스터를 읽음.
 * "nRF24 초기화 실패" 원인이 배선인지 라이브러리 설정인지 구분한다.
 *
 * 로봇 배선:
 *   CSN → A1,  SCK → A2,  MOSI(MO) → A3,  MISO(MI) → A4
 *   CE  → 5V 직결,  VCC → 5V,  GND → GND
 *
 * 기대 결과:
 *   STATUS = 0x0E  →  SPI 정상, 배선 OK (라이브러리 설정 문제)
 *   STATUS = 0xFF  →  MISO 플로팅 = 배선 오류 (MOSI/MISO 스왑 의심)
 *   STATUS = 0x00  →  MISO 고착 LOW = 배선 오류
 */

#define PIN_CSN  A1
#define PIN_SCK  A2
#define PIN_MOSI A3
#define PIN_MISO A4

// ── 비트뱅잉 SPI (MODE 0, MSB first) ──────────────────────────
static uint8_t spi_xfer(uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_MOSI, (out >> i) & 1);
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(2);
    in = (in << 1) | digitalRead(PIN_MISO);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(2);
  }
  return in;
}

// nRF24 레지스터 읽기 (R_REGISTER = 0x00 | addr)
static uint8_t nrf_read_reg(uint8_t addr) {
  digitalWrite(PIN_CSN, LOW);
  spi_xfer(addr & 0x1F);          // 커맨드 (STATUS가 반환됨)
  uint8_t val = spi_xfer(0xFF);   // 더미 바이트 → 레지스터 값
  digitalWrite(PIN_CSN, HIGH);
  return val;
}

// NOP 커맨드 → STATUS 레지스터만 읽기
static uint8_t nrf_status() {
  digitalWrite(PIN_CSN, LOW);
  uint8_t s = spi_xfer(0xFF);     // NOP = 0xFF
  digitalWrite(PIN_CSN, HIGH);
  return s;
}

// ── 핀 비트뱅잉으로 MISO=A3, MOSI=A4 스왑 버전 테스트 ─────────
#define PIN_MOSI_SWAP A4
#define PIN_MISO_SWAP A3

static uint8_t spi_xfer_swap(uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_MOSI_SWAP, (out >> i) & 1);
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(2);
    in = (in << 1) | digitalRead(PIN_MISO_SWAP);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(2);
  }
  return in;
}

static uint8_t nrf_status_swap() {
  // MOSI/MISO 스왑 배선 가정으로 동일 테스트
  pinMode(PIN_MOSI_SWAP, OUTPUT);
  pinMode(PIN_MISO_SWAP, INPUT);
  digitalWrite(PIN_CSN, LOW);
  uint8_t s = spi_xfer_swap(0xFF);
  digitalWrite(PIN_CSN, HIGH);
  // 원래대로 복구
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  return s;
}

// ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_CSN,  OUTPUT); digitalWrite(PIN_CSN,  HIGH);
  pinMode(PIN_SCK,  OUTPUT); digitalWrite(PIN_SCK,  LOW);
  pinMode(PIN_MOSI, OUTPUT); digitalWrite(PIN_MOSI, LOW);
  pinMode(PIN_MISO, INPUT);

  delay(100);   // nRF24 전원 안정화 대기

  Serial.println(F("=== nRF24 베어메탈 SPI 진단 ==="));
  Serial.println(F("CSN=A1  SCK=A2  MOSI=A3  MISO=A4"));
  Serial.println();

  // ── 테스트 1: STATUS (NOP) ─────────────────────────────────
  uint8_t status = nrf_status();
  Serial.print(F("STATUS (NOP)   = 0x"));
  Serial.print(status, HEX);
  if (status == 0x0E) {
    Serial.println(F("  ✔  정상 — SPI 통신 OK"));
  } else if (status == 0xFF) {
    Serial.println(F("  ✘  MISO 플로팅 — 배선 오류 (아래 스왑 결과 확인)"));
  } else if (status == 0x00) {
    Serial.println(F("  ✘  MISO 고착 LOW — 배선 오류"));
  } else {
    Serial.println(F("  ?  비정상 값 (부분 연결?)"));
  }

  // ── 테스트 2: CONFIG 레지스터 읽기 (주소 0x00, 기본값 0x08) ──
  uint8_t cfg = nrf_read_reg(0x00);
  Serial.print(F("CONFIG (0x00)  = 0x"));
  Serial.print(cfg, HEX);
  if (cfg == 0x08) {
    Serial.println(F("  ✔  기본값 정상"));
  } else if (cfg == 0xFF || cfg == 0x00) {
    Serial.println(F("  ✘  통신 실패"));
  } else {
    Serial.print(F("  (리셋됐거나 이전에 기록된 값)"));
    Serial.println();
  }

  // ── 테스트 3: RF_CH 레지스터 (0x05, 기본값 0x02) ─────────────
  uint8_t rfch = nrf_read_reg(0x05);
  Serial.print(F("RF_CH (0x05)   = 0x"));
  Serial.print(rfch, HEX);
  if (rfch == 0x02) {
    Serial.println(F("  ✔  기본값 정상"));
  } else {
    Serial.println();
  }

  // ── 테스트 4: MOSI/MISO 스왑 버전 ────────────────────────────
  Serial.println();
  Serial.println(F("--- MOSI↔MISO 스왑 테스트 (MO=A4, MI=A3) ---"));
  uint8_t status_swap = nrf_status_swap();
  Serial.print(F("STATUS (스왑)  = 0x"));
  Serial.print(status_swap, HEX);
  if (status_swap == 0x0E) {
    Serial.println(F("  ✔  스왑하면 정상 → MO/MI 핀 바꿔 꽂으세요!"));
  } else {
    Serial.println(F("  ✘  스왑해도 실패"));
  }

  Serial.println();
  Serial.println(F("=== 판정 ==="));
  if (status == 0x0E) {
    Serial.println(F("배선 정상. RF24 라이브러리 설정 문제 → CE 핀 수정 필요."));
  } else if (status_swap == 0x0E) {
    Serial.println(F("MO→A4, MI→A3으로 바꿔 꽂으세요."));
  } else {
    Serial.println(F("SPI 통신 실패. 아래 항목 확인:"));
    Serial.println(F("  1) VCC=5V, GND 연결 확인"));
    Serial.println(F("  2) CSN=A1, SCK=A2 핀 확인"));
    Serial.println(F("  3) 어댑터 보드 전원 LED 켜져 있는지 확인"));
  }
}

void loop() {
  // 매 3초마다 STATUS 재측정 (전원 문제 확인용)
  delay(3000);
  uint8_t s = nrf_status();
  Serial.print(F("STATUS = 0x")); Serial.println(s, HEX);
}
