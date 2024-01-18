#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <Arduino.h>

void loraWanInit();
void loraSendDate(uint8_t *bufPtr, uint8_t data_len);
void loraSendTempDate(uint8_t len, uint8_t *buf);

#endif
