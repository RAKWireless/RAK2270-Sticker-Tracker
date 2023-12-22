#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include "service_lora.h"

typedef enum
{
  NOT_JOINED = 0x0,
  JOINED = 0x1,
  SEND_OK = 0x2,
  SEND_NG = 0x3,
} LORA_NETWORK_STAUS_t;

void loraWanInit();
void loraSendDate();
void loraSendTempDate(uint8_t len, uint8_t *buf);
uint8_t get_lora_network_status(void);
void send_data(void);

#define DEFAULT_TRIGGER_REJION_INTERVERAL   (12*60*60*1000) //12hr, 12*60*60*1000
#define DEFAULT_LINKCHECK_INTERVAL          (1*60*60*1000)  //1hr, 1*60*60*1000

#endif
