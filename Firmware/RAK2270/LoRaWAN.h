#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <Arduino.h>


typedef enum
{
  DLCMD_DR = 0x00,                   //LoRaWAN, DR
  DLCMD_ADR = 0x01,                  //LoRaWAN, ADR
  DLCMD_UL_INTERVAL = 0x02,          //Period uplink interval
  DLCMD_MD_ENABLE = 0x03,            //Motion detect, enable to detect
  DLCMD_MD_THRESHOLD = 0x04,         //Motion detect, the threshold of motion detection
  DLCMD_MD_SAMPLE_RATE = 0x05,       //Motion detect, the sample rate of lis3dh
  DLCMD_CFM = 0x06,                  //LoRaWAN, use confirmed packet
  DLCMD_MD_DURATION = 0x07,          //Motion detect, the detected duration for event trigger
  DLCMD_MD_NUMBER = 0x08,            //Motion detect, the detected number for event trigger
  DLCMD_MD_SILENT_PERIOD = 0x09,      //Motion detect, the silent period after event trigger
  DLCMD_REJION = 0x0A,                //LORAWAN, do re-join
  DLCMD_LINKCHECK_TIMEOUT = 0x0B,    //LORAWAN, link check timeout for trigger to re-join
} DOWNLINK_CMD_ID_t;

typedef struct DOWNLINK_CMD_s
{
  uint8_t length;
  uint8_t command;
  uint8_t data_8;
  uint16_t data_16;
  uint32_t data_32;
} DOWNLINK_CMD_t;

typedef struct DOWNLINK_ACK_s
{
  uint8_t length;
  uint8_t data[16];
} DOWNLINK_ACK_t;

typedef struct RETURN_PRE_SETTING_s
{
  bool check;
  bool enable;
  DOWNLINK_CMD_ID_t cmd;
  uint8_t data_8;
  //uint16_t data_16;
  //uint32_t data_32;
} RETURN_PRE_SETTING_t;

void loraWanInit();
void loraSendData(uint8_t *bufPtr, uint8_t data_len);
void loraSendTempData(uint8_t len, uint8_t *buf);
void loraSendMDEvent();
void reset_linkcheck_start();
void reset_last_send_ok_time();
void downlink_cmd_handle();

extern bool has_downlink_cmd;
extern RETURN_PRE_SETTING_t g_return_pre_setting;
extern bool f_cmd_rejoin;

typedef enum
{
  NOT_JOINED = 0x0,
  JOINED = 0x1,
  SEND_OK = 0x2,
  SEND_NG = 0x3,
} LORA_NETWORK_STAUS_t;


#endif
