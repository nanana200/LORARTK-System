#include "lora_radio.h"

#include <SPI.h>

#include "config.h"

namespace {
constexpr uint8_t WRITE_BIT = 0x80;
constexpr uint8_t READ_BIT = 0x00;
constexpr uint8_t MODE_SLEEP_LORA = 0x80;
constexpr uint8_t MODE_STANDBY_LORA = 0x81;
constexpr uint8_t MODE_TX_LORA = 0x83;
constexpr uint8_t MODE_RX_CONTINUOUS_LORA = 0x85;
constexpr uint8_t NO_MORE = 0xFF;

SPISettings radio_spi_settings(LORA_SPI_HZ, MSBFIRST, SPI_MODE0);
uint32_t last_rssi_report_ms = 0;

void print_hex(const uint8_t *buffer, uint8_t length) {
  if (buffer == nullptr) {
    return;
  }
  for (uint16_t i = 0; i < length; ++i) {
    Serial.printf("%02X", buffer[i]);
  }
}
}  // namespace

void lora_write(uint8_t address, uint8_t value) {
  SPI.beginTransaction(radio_spi_settings);
  digitalWrite(LORA_PIN_CS, LOW);
  SPI.transfer(static_cast<uint8_t>(WRITE_BIT | address));
  SPI.transfer(value);
  digitalWrite(LORA_PIN_CS, HIGH);
  SPI.endTransaction();
}

void lora_write_burst(uint8_t address, uint8_t data1, uint8_t data2,
                      uint8_t data3) {
  SPI.beginTransaction(radio_spi_settings);
  digitalWrite(LORA_PIN_CS, LOW);
  SPI.transfer(static_cast<uint8_t>(WRITE_BIT | address));
  SPI.transfer(data1);
  SPI.transfer(data2);
  if (data3 != NO_MORE) {
    SPI.transfer(data3);
  }
  digitalWrite(LORA_PIN_CS, HIGH);
  SPI.endTransaction();
}

uint8_t lora_read(uint8_t address) {
  SPI.beginTransaction(radio_spi_settings);
  digitalWrite(LORA_PIN_CS, LOW);
  SPI.transfer(static_cast<uint8_t>(READ_BIT | address));
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(LORA_PIN_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void lora_setup() {
  pinMode(LORA_PIN_EN, OUTPUT);
  pinMode(LORA_PIN_RESET, OUTPUT);
  pinMode(LORA_PIN_CS, OUTPUT);
  pinMode(LORA_PIN_DIO0, INPUT);

  digitalWrite(LORA_PIN_CS, HIGH);
  digitalWrite(LORA_PIN_EN, HIGH);
  delay(10);

  // STM 보드에서 EN=High, RESET=High로 사용한 SX1276 모듈을 확실히 재시작한다.
  digitalWrite(LORA_PIN_RESET, LOW);
  delay(10);
  digitalWrite(LORA_PIN_RESET, HIGH);
  delay(20);

  SPI.begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_CS);

  lora_write(0x09, 0x8F);  // PA_BOOST, 17 dBm
  lora_write(0x0B, 0x31);  // OCP 약 140 mA
  lora_write(0x11, 0x00);  // IRQ mask 없음
  lora_write(0x12, 0xFF);  // 모든 IRQ clear
}

void lora_freq() {
  lora_write(0x01, MODE_SLEEP_LORA);
  delay(10);

  // 922.1 MHz: FRF = 0xE68666 (STM/RTKTTGO와 동일)
  lora_write_burst(0x06, 0xE6, 0x86, 0x66);
}

void packet_set() {
  lora_write(0x01, MODE_STANDBY_LORA);
  lora_write(0x1D, 0x72);  // BW125 kHz, CR 4/5, Explicit header
  lora_write(0x1E, 0x74);  // SF7, Payload CRC ON
  lora_write(0x20, 0x00);  // Preamble MSB
  lora_write(0x21, 0x08);  // Preamble 8 symbols
}

void fifo_set() {
  lora_write(0x01, MODE_STANDBY_LORA);
  lora_write_burst(0x0E, 0x80, 0x00, NO_MORE);  // TX base 0x80, RX base 0x00
}

void rx_set() {
  lora_write(0x01, MODE_STANDBY_LORA);
  lora_write(0x12, 0xFF);
  lora_write(0x0F, 0x00);
  lora_write(0x0D, 0x00);
  lora_write(0x40, 0x00);  // DIO0 = RxDone
  lora_write(0x01, MODE_RX_CONTINUOUS_LORA);
}

bool lora_send_packet(const uint8_t *buffer, uint8_t length,
                      uint32_t timeout_ms) {
  if (buffer == nullptr || length == 0) {
    return false;
  }

  lora_write(0x01, MODE_STANDBY_LORA);
  lora_write(0x12, 0xFF);
  lora_write(0x0D, 0x80);  // RegFifoAddrPtr = TX base

  SPI.beginTransaction(radio_spi_settings);
  digitalWrite(LORA_PIN_CS, LOW);
  SPI.transfer(0x80);  // RegFifo write
  for (uint16_t i = 0; i < length; ++i) {
    SPI.transfer(buffer[i]);
  }
  digitalWrite(LORA_PIN_CS, HIGH);
  SPI.endTransaction();

  lora_write(0x22, length);  // RegPayloadLength
  lora_write(0x40, 0x40);    // DIO0 = TxDone
  lora_write(0x01, MODE_TX_LORA);

  const uint32_t started_ms = millis();
  while ((lora_read(0x12) & 0x08U) == 0U) {
    if (static_cast<uint32_t>(millis() - started_ms) >= timeout_ms) {
      lora_write(0x01, MODE_STANDBY_LORA);
      lora_write(0x12, 0xFF);
      return false;
    }
    yield();
  }

  lora_write(0x01, MODE_STANDBY_LORA);
  lora_write(0x12, 0xFF);
  return true;
}

int lora_receive_packet(uint8_t *buffer, uint8_t *length,
                        uint8_t *irq_flags) {
  if (buffer == nullptr || length == nullptr || irq_flags == nullptr) {
    return 0;
  }

  const uint8_t irq = lora_read(0x12);
  *irq_flags = irq;
  *length = 0;

  if ((irq & 0x20U) != 0U) {  // PayloadCrcError
    lora_write(0x01, MODE_STANDBY_LORA);
    lora_write(0x12, 0xFF);
    return -1;
  }
  if ((irq & 0x40U) == 0U) {  // RxDone 없음
    return 0;
  }

  lora_write(0x01, MODE_STANDBY_LORA);
  const uint8_t received_length = lora_read(0x13);
  const uint8_t current_address = lora_read(0x10);
  lora_write(0x0D, current_address);

  // STM 버전과 같이 FIFO를 한 번의 SPI transaction으로 복사한다.
  SPI.beginTransaction(radio_spi_settings);
  digitalWrite(LORA_PIN_CS, LOW);
  SPI.transfer(0x00);  // RegFifo read
  for (uint16_t i = 0; i < received_length; ++i) {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(LORA_PIN_CS, HIGH);
  SPI.endTransaction();

  *length = received_length;
  lora_write(0x12, 0xFF);
  return 1;
}

int16_t lora_current_rssi_dbm() {
  // 922.1 MHz는 HF port이므로 RSSI offset은 -157 dB.
  return static_cast<int16_t>(lora_read(0x1B)) - 157;
}

int16_t lora_packet_rssi_dbm() {
  return static_cast<int16_t>(lora_read(0x1A)) - 157;
}

int16_t lora_packet_snr_x100() {
  const int8_t raw = static_cast<int8_t>(lora_read(0x19));
  return static_cast<int16_t>(raw) * 25;
}

void rssi_graph_packet(uint8_t crc_ok, const uint8_t *buffer,
                       uint8_t length) {
  Serial.printf("P,%lu,%d,%d,%u,", static_cast<unsigned long>(millis()),
                lora_packet_rssi_dbm(), lora_packet_snr_x100(),
                crc_ok ? 1U : 0U);
  if (crc_ok != 0U) {
    print_hex(buffer, length);
  }
  Serial.print("\r\n");
}

void uart_send_noise_rssi() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - last_rssi_report_ms) <
      RSSI_REPORT_PERIOD_MS) {
    return;
  }
  last_rssi_report_ms = now;
  Serial.printf("R,%lu,%d\r\n", static_cast<unsigned long>(now),
                lora_current_rssi_dbm());
}

void lora_rtcm_debug_result(uint8_t sequence, uint16_t rtcm_type,
                            uint16_t frame_length, uint8_t crc_ok,
                            uint8_t fragment_count, uint8_t forwarded,
                            const char *status) {
  Serial.printf("T,%lu,%u,%u,%u,%u,%u,%u,%s\r\n",
                static_cast<unsigned long>(millis()), sequence, rtcm_type,
                frame_length, crc_ok ? 1U : 0U, fragment_count,
                forwarded ? 1U : 0U, status == nullptr ? "UNKNOWN" : status);
}
