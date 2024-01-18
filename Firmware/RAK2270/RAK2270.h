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

void period_handler(void);

/*
 * For Test and Debug
 */
#define ENABLE_TEMPERTURE_TEST      (0)
#define ENABLE_FACTORY_MODE_SLEEP   (1)

#endif
