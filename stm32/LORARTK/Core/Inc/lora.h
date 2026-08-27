#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"  // 이걸 넣어야 GPIO_PIN_RESET 같은 거 인식함
#include "stdio.h"
#include "string.h"

// 1. 매크로 정의 (main.c에 있던 거 다 가져옴)
#define write_bit 0x80
#define read_bit 0x00
#define sleep 0x80 // 레지스터 설정중에서 LORA모드 변경 + SLEEP 상태(SPI만됨, RF정지)에서 설정할수있는 모드,레지스터가있음,저전력모드. 즉, 시동을 끄고 좀 설정하는 느낌
                   // 전원을 처음켜면 FSK/OSK모드여서 이걸 LORA모드로 사용하기 위해서는 무조건 SLEEP상태에서만 가능(stanby불가능) 그래서 전원 꼽고 이거 한번 켜주고 레지스터 설정해야 lora모드의 레지스터의 주소로 값이 들어감
#define stanby 0x81 // RF 송수신은 아직 안 하지만, 클럭 켜지고, 전력좀 쓰고. 즉, 시동키고 대기하는 느낌
#define no_more 0xFF // burst accecss에서 2개이상으로 하고싶지는 않을때


// [중요] main.c에 있는 핸들러들을 '빌려온다'고 선언해야 함
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;



////////////////////////////////////////////////////////////////////////////////////////////////////// 3. 함수 프로토타입 (남들이 쓸 함수만)
void lora_log(const char *fmt, ...);
void lora_setup(void);
void lora_write(uint8_t adr,uint8_t data);
void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3);
uint8_t lora_read(uint8_t adr);
void lora_freq(void);
void packet_set(void);
void rx_set(void);
void rx_read(void);
int lora_receive_packet(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags);
int16_t lora_packet_rssi_dbm(void);
int16_t lora_packet_snr_x100(void);
void rssi_graph(void);
void uart_send_noise_rssi(void);
void rssi_graph_packet(uint8_t crc_ok, const uint8_t *buffer, uint8_t length);
void lora_rtcm_debug_result(uint8_t sequence, uint16_t rtcm_type,
                            uint16_t frame_length, uint8_t crc_ok,
                            uint8_t fragment_count, uint8_t forwarded,
                            const char *status);

void fifo_set(void);


#endif /* INC_LORA_H_ */


