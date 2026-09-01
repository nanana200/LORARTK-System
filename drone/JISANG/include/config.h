#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// ESP32-S3 <-> SX1276 wiring (사용자가 제공한 배선표)
// -----------------------------------------------------------------------------
constexpr int LORA_PIN_CS    = 8;   // CSn
constexpr int LORA_PIN_SCK   = 12;  // CLK
constexpr int LORA_PIN_MISO  = 13;
constexpr int LORA_PIN_MOSI  = 11;
constexpr int LORA_PIN_EN    = 6;
constexpr int LORA_PIN_RESET = 14;
constexpr int LORA_PIN_DIO0  = 38;  // G0 / DIO0 / RxDone

// ESP32-S3 <-> UM982 UART.
// 사진에는 UM982 배선이 없으므로 LoRa와 겹치지 않는 핀을 기본값으로 정했다.
// 실제 배선이 다르면 아래 두 값만 변경한다.
constexpr int UM982_RX_PIN = 18;  // ESP32 RX  <- UM982 TX
constexpr int UM982_TX_PIN = 17;  // ESP32 TX  -> UM982 RX (RTCM)

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t UM982_BAUD = 115200;

// true이면 부팅할 때 UM982 현재 UART 포트에 GGA 10 Hz 명령을 보낸다.
// Flash 저장은 하지 않으므로 부팅 때마다 적용하며 SAVECONFIG 반복 기록을 피한다.
constexpr bool UM982_SET_GGA_10HZ_AT_BOOT = true;
constexpr uint8_t UM982_EXPECTED_GGA_HZ = 10;

// 매 GGA마다 RAW와 해석 로그를 모두 출력하면, UM982가 여러 메시지를 10 Hz로
// 출력할 때 USB debug가 병목이 될 수 있다. 해석된 [GGA] 로그는 항상 출력하고
// 원문까지 필요할 때만 true로 바꾼다.
constexpr bool UM982_PRINT_RAW_GGA = false;

// STM32 LORARTK/지상 RTKTTGO와 반드시 같아야 하는 무선 설정.
constexpr uint32_t LORA_SPI_HZ = 8000000;
constexpr uint32_t LORA_FREQUENCY_HZ = 922100000;
constexpr uint32_t RSSI_REPORT_PERIOD_MS = 50;
// A2 첫 fragment부터 마지막 fragment까지 기다리는 최대 시간.
constexpr uint32_t FRAGMENT_TIMEOUT_MS = 1000;
