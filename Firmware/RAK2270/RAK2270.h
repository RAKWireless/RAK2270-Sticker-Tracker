/**
   @file  RAK2270.h
   @author rakwireless.com
   @brief Used to include required header files.
   @version 0.1
   @date 2023-10-3
   @copyright Copyright (c) 2023
**/
#ifndef __RAK2270_H__
#define __RAK2270_H__


#include <Arduino.h>

typedef enum
{
  ACTIVATE_STAGE = 0x0,
  JOIN_STAGE = 0x1,
  PERIODIC_UPLINK_STAGE = 0x2,
} SYSTEM_STAGE_t;

uint8_t get_system_stage();
void goto_join_stage();
void goto_periodic_uplink_stage();
void return_to_join_stage();
void join_handler(void);
void periodic_uplink_handler(void);


#define TIMER_PERIODIC_UPLINK   RAK_TIMER_0
#define TIMER_MD_SILENT_PERIOD  RAK_TIMER_1
#define TIMER_DOWNLINK_CHECK    RAK_TIMER_2

#define ACTIVATE_DEBOUNCE_TIMEOUT 5000

#endif
