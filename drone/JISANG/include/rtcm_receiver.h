#pragma once

#include <Arduino.h>

void LORARTK_Begin(HardwareSerial &um982_serial);
void IRAM_ATTR LORARTK_NotifyRadioInterrupt();
void LORARTK_Process();
void LORARTK_SetDetection(uint8_t detected, uint8_t person_count);
