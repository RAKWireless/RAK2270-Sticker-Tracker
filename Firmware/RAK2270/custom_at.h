#ifndef __CUSTOM_AT_H__
#define __CUSTOM_AT_H__

#include <Arduino.h>

/** Settings offset in flash */
#define STORE_DATA_OFFSET     (0x8E00)
//#define SEND_FREQ_OFFSET    0x00000000
#define SEND_FREQ_OFFSET       (0x0+STORE_DATA_OFFSET)

#define DEFAULT_TXINTERVAL  60

bool init_frequency_at(void);

bool get_at_setting(uint32_t setting_type);
bool save_at_setting(uint32_t setting_type);

extern uint32_t  g_txInterval;


//#define FACTORY_MODE_OFFSET 0x00000005
#define FACTORY_MODE_OFFSET     (0x5+STORE_DATA_OFFSET)
typedef enum
{
  FACTORY_MODE_OFF = 0x0,
  FACTORY_MODE_ON = 0x1
} FACTORY_MODE_t;
#define DEFAULT_FACTORY_MODE  0xFF

void init_factory_mode();
uint8_t get_factory_mode(void);
void register_factory_mode_atcmd(void);
#endif

#define EVENT_MODE_OFFSET     (0x6+STORE_DATA_OFFSET)
#define EVENT_RETRIEVAL_UPLINK  (0x1)
#define EVENT_MOTION_DETECTION  (0x2)
#define EVENT_SHOCK_DETECTION   (0x4)
#define DEFAULT_EVENT_MODE      (0|EVENT_RETRIEVAL_UPLINK|EVENT_MOTION_DETECTION)

void init_event_mode();
uint8_t get_event_mode(void);
bool set_event_mode(uint8_t event_mode);
