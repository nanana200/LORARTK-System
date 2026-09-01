#include <Arduino.h>

#include "config.h"
#include "rtcm_receiver.h"

HardwareSerial UM982Serial(1);

void IRAM_ATTR onLoRaDio0() { LORARTK_NotifyRadioInterrupt(); }

void setup() {
  Serial.begin(DEBUG_BAUD);
  const uint32_t serial_wait_start = millis();
  while (!Serial && static_cast<uint32_t>(millis() - serial_wait_start) < 3000) {
    delay(10);
  }

  // UM982가 GGA를 포함한 여러 메시지를 10 Hz로 출력하는 경우에도 burst를
  // 놓치지 않도록 RX 여유를 크게 잡는다. RTCM 송신 버퍼는 최대 frame의 3배 이상.
  UM982Serial.setRxBufferSize(8192);
  UM982Serial.setTxBufferSize(4096);
  UM982Serial.begin(UM982_BAUD, SERIAL_8N1, UM982_RX_PIN, UM982_TX_PIN);

  LORARTK_Begin(UM982Serial);
  attachInterrupt(digitalPinToInterrupt(LORA_PIN_DIO0), onLoRaDio0, RISING);

  if (UM982_SET_GGA_10HZ_AT_BOOT) {
    delay(100);
    UM982Serial.print("GPGGA 0.1\r\n");
    Serial.print("[UM982 CFG] Sent GPGGA 0.1 (10 Hz, current UM982 port)\r\n");
  }
}

void loop() {
  LORARTK_Process();
  yield();
}
