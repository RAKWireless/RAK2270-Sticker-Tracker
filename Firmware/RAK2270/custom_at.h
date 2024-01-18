#ifndef __CUSTOM_AT_H__
#define __CUSTOM_AT_H__

#include <Arduino.h>
#include "./custom.h"

/* The offset of configure in flash memory */
#define SEND_FREQ_OFFSET      (0x0)
#define FACTORY_MODE_OFFSET   (0x5)
#define EVENT_MODE_OFFSET     (0x6)

/* Sending frequency for periodic uplink and join */
extern uint32_t  g_txInterval;

bool init_frequency_at(void);
bool get_at_setting(uint32_t setting_type);
bool save_at_setting(uint32_t setting_type);

/* Enable or disable for factory mode */
typedef enum
{
  FACTORY_MODE_OFF = 0x0,
  FACTORY_MODE_ON = 0x1
} FACTORY_MODE_t;

uint8_t get_factory_mode(void);
void init_factory_mode();

/* The default value of configure */
#define DEFAULT_TXINTERVAL    INTERVAL_PERIODIC_UPLINK
#define DEFAULT_FACTORY_MODE  0xFF

#endif
