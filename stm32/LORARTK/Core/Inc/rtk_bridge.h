#ifndef INC_RTK_BRIDGE_H_
#define INC_RTK_BRIDGE_H_

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void RTK_Bridge_Init(void);
void RTK_Bridge_Process(void);
bool RTK_Bridge_ForwardRTCM(const uint8_t *frame, uint16_t length);
void Debug_Log(const char *fmt, ...);

uint32_t RTK_Bridge_GetForwardedFrames(void);
uint32_t RTK_Bridge_GetForwardedBytes(void);
uint32_t RTK_Bridge_GetDroppedFrames(void);
uint32_t RTK_Bridge_GetUM982Bytes(void);

#endif /* INC_RTK_BRIDGE_H_ */
