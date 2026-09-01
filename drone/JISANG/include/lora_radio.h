#pragma once

#include <Arduino.h>

// 기존 STM/TTGO에서 사용한 함수 이름을 그대로 유지한다.
void lora_setup();
void lora_write(uint8_t address, uint8_t value);
void lora_write_burst(uint8_t address, uint8_t data1, uint8_t data2,
                      uint8_t data3);
uint8_t lora_read(uint8_t address);
void lora_freq();
void packet_set();
void fifo_set();
void rx_set();

int lora_receive_packet(uint8_t *buffer, uint8_t *length,
                        uint8_t *irq_flags);
int16_t lora_current_rssi_dbm();
int16_t lora_packet_rssi_dbm();
int16_t lora_packet_snr_x100();

void rssi_graph_packet(uint8_t crc_ok, const uint8_t *buffer,
                       uint8_t length);
void uart_send_noise_rssi();
void lora_rtcm_debug_result(uint8_t sequence, uint16_t rtcm_type,
                            uint16_t frame_length, uint8_t crc_ok,
                            uint8_t fragment_count, uint8_t forwarded,
                            const char *status);

