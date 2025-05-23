#ifndef __CUSTOM_AT_H__
#define __CUSTOM_AT_H__

#include <Arduino.h>
#include "./custom.h"

/* The offset of configure in flash memory */
#define SEND_FREQ_OFFSET              (0x0) //5
#define FACTORY_MODE_OFFSET           (0x5) //1
#define NTC_CALIBRATION_MODE_OFFSET   (0x6) //1
#define MD_ENABLE_OFFSET              (0x8) //1
#define MD_THRESHOLD_OFFSET           (0xC) //2
#define MD_DURATION_OFFSET            (0x10) //2
#define MD_NUMBER_OFFSET              (0x14) //1
#define MD_SILENT_PERIOD_OFFSET       (0x18) //2
#define MD_SAMPLE_RATE_OFFSET         (0x1A) //2
#define JOIN_INTERVAL_OFFSET          (0x1C) //4
#define LINKCHECK_TIMEOUT_OFFSET     (0x20) //4


/* Sending frequency for periodic uplink */
extern uint32_t  g_txInterval;

#define DEFAULT_TXINTERVAL            INTERVAL_PERIODIC_UPLINK //60 mins

bool init_frequency_at(void);
bool get_at_setting(uint32_t setting_type);
bool save_at_setting(uint32_t setting_type);

/* operate mode */
#define DEFAULT_FACTORY_MODE          FACTORY_MODE_ON
#define DEFAULT_NTC_CALIBRATION_MODE  0x0
#define DEFAULT_DEVELOPMENT_MODE      0x0

/* Enable or disable for factory mode */
typedef enum
{
  FACTORY_MODE_OFF = 0x0,
  FACTORY_MODE_ON = 0x1
} FACTORY_MODE_t;

uint8_t get_factory_mode(void);
void init_factory_mode();
uint8_t get_ntc_calibration_mode();
void init_ntc_calibration_mode();
uint8_t get_development_mode();
void init_development_mode();

/* Join and re-join */
#define DEFAULT_JOIN_INTERVAL         INTERVAL_JOIN //480 mins (1-1440)
#define DEFAULT_LINKCHECK_TIMEOUT   0 //0: disable, v1.1.0=12*60 mins (0-1440)

void register_rejoin_atcmd();
uint32_t get_join_interval();
void init_join_interval();
uint32_t get_linkcheck_timeout();
bool set_linkcheck_timeout(uint32_t timeout);
void init_linkcheck_timeout();

/* Motion detection */
#define DEFAULT_MD_ENABLE             0
#define DEFAULT_MD_THRESHOLD          400 //400 mg (50~16000)
#define DEFAULT_MD_SAMPLE_RATE        1 //10 Hz (0: 1 HZ, 1: 10 Hz)
#define DEFAULT_MD_DURATION           1000 //1000 ms (300~15000) 
#define DEFAULT_MD_NUMBER             2 //2 (1~10)
#define DEFAULT_MD_SILENT_PERIOD      3600 //10 s (10~3600)

uint8_t get_md_enable(void);
bool set_md_enable(uint8_t enable);
uint16_t get_md_threshold(void);
bool set_md_threshold(uint16_t threshold);
uint16_t get_md_sample_rate(void);
bool set_md_sample_rate(uint16_t sample_rate);
uint16_t get_md_duration(void);
bool set_md_duration(uint16_t duration);
uint8_t get_md_number(void);
bool set_md_number(uint8_t number);
uint16_t get_md_silent_period(void);
bool set_md_silent_period(uint16_t silent_period);
void init_md_atcmd();

#endif
