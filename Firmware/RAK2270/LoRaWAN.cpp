#include "service_lora.h"
#include "udrv_flash.h"
#include "./custom_at.h"
#include "./LoRaWAN.h"
#include "./RAK2270.h"
#include "./lis3dh_md.h"
#include "./ntc.h"


static uint8_t retry = LORAWAN_RETRY;

/** Packet buffer for sending */
uint8_t collected_data[64] = { 0 };

bool has_downlink_cmd = false;
DOWNLINK_CMD_t downlink_cmd;
DOWNLINK_ACK_t downlink_ack;
RETURN_PRE_SETTING_t g_return_pre_setting;
bool f_cmd_rejoin = false;

bool send_downlink_ack(bool confirmed_ack)
{
  if (api.lorawan.send(downlink_ack.length, downlink_ack.data, FPORT_DOWNLINK_ACK, confirmed_ack, retry)) {
    Serial.println("Sending is requested");
    return true;
  }
  else {
    Serial.println("Sending failed");
    return false;
  }
}

void downlink_cmd_handle()
{
  g_return_pre_setting.check = false;
  g_return_pre_setting.enable = false;
  has_downlink_cmd = false;

  switch (downlink_cmd.command)
  {
    case DLCMD_DR: // Setting data rate
    {
      uint8_t dr_org = api.lorawan.dr.get();
      if (!api.lorawan.dr.set(downlink_cmd.data_8)) {
        Serial.printf("DLCMD_DR(%02X), Failed to set DR to %d!\r\n", DLCMD_DR, downlink_cmd.data_8);
      }
      else {
        Serial.printf("Set DR to %d\r\n", downlink_cmd.data_8);
        g_return_pre_setting.check = true;
        g_return_pre_setting.cmd = DLCMD_DR;
        g_return_pre_setting.data_8 = dr_org;
        send_downlink_ack(true);
      }
      return;
    }
    case DLCMD_ADR: // Setting adr
    {
      uint8_t adr_org = api.lorawan.adr.get();
      if (!api.lorawan.adr.set(downlink_cmd.data_8)) {
        Serial.printf("DLCMD_ADR(%02X), Failed to set ADR to %d!\r\n", DLCMD_ADR, downlink_cmd.data_8);
      }
      else {
        Serial.printf("Set ADR to %d\r\n", downlink_cmd.data_8);
        g_return_pre_setting.check = true;
        g_return_pre_setting.cmd = DLCMD_ADR;
        g_return_pre_setting.data_8 = adr_org;
        send_downlink_ack(true);
      }
      return;
    }
    case DLCMD_UL_INTERVAL: // Modify the interval of periodic uplink, range: 1~1440 mintues
    {
      g_txInterval = (uint32_t)downlink_cmd.data_16;
      save_at_setting(SEND_FREQ_OFFSET);
      Serial.printf("Set period uplink interval = %d\r\n",g_txInterval);

      api.system.timer.stop(TIMER_PERIODIC_UPLINK);
      while (api.system.timer.create(TIMER_PERIODIC_UPLINK, (RAK_TIMER_HANDLER)periodic_uplink_handler, RAK_TIMER_PERIODIC) != true) {
        Serial.printf("Failed to creat timer.\r\n");
        delay(100);
      }
      while (api.system.timer.start(TIMER_PERIODIC_UPLINK, g_txInterval * 60000, NULL) != true) {
        Serial.printf("Failed to start timer.\r\n");
        delay(100);
      }
      return;
    }
    case DLCMD_MD_ENABLE: // Setting md_enable
    {
      if (get_md_enable() != downlink_cmd.data_8) {
        set_md_enable(downlink_cmd.data_8);
        if (get_md_enable()) {
          lis3dh_md_begin();
        }
        else {
          lis3dh_md_end();
        }
      }
      Serial.printf("Set md enable = %d\r\n", downlink_cmd.data_8);
      return;
    }
    case DLCMD_MD_THRESHOLD: // Setting md_threshold
    {
      if (get_md_threshold() != downlink_cmd.data_16) {
        set_md_threshold(downlink_cmd.data_16);
        if (get_md_enable()) {
          lis3dh_md_pause();
          lis3dh_md_resume();
        }
      }
      Serial.printf("Set md threshold = %d\r\n", downlink_cmd.data_16);
      return;
    }
    case DLCMD_MD_SAMPLE_RATE: // Setting md_sample_rate
    {
      if (get_md_sample_rate() != downlink_cmd.data_16) {
        set_md_sample_rate(downlink_cmd.data_16);
        if (get_md_enable()) {
          lis3dh_md_pause();
          lis3dh_md_resume();
        }
      }
      Serial.printf("Set md sample rate = %d\r\n", downlink_cmd.data_16);
      return;
    }
    case DLCMD_CFM: // Enable or disable for sending confirmed package
    {
      if (!api.lorawan.cfm.set(downlink_cmd.data_8))
        Serial.printf("DLCMD_CFM(%02X), Failed to set CFM to %d!\r\n", DLCMD_CFM, downlink_cmd.data_8);
      else {
        Serial.printf("Set CFM to %d\r\n", downlink_cmd.data_8);
      }
      return;
    }
    case DLCMD_MD_DURATION: // Setting md_duration
    {
      if (get_md_duration() != downlink_cmd.data_16) {
        set_md_duration(downlink_cmd.data_16);
        if (get_md_enable()) {
          lis3dh_md_pause();
          lis3dh_md_resume();
        }
      }
      Serial.printf("Set md duration = %d\r\n", downlink_cmd.data_16);
      return;
    }
    case DLCMD_MD_NUMBER: // Setting md_number
    {
      if (get_md_number() != downlink_cmd.data_8) {
        set_md_number(downlink_cmd.data_8);
        if (get_md_enable()) {
          lis3dh_md_pause();
          lis3dh_md_resume();
        }
      }
      Serial.printf("Set md number = %d\r\n", downlink_cmd.data_8);
      return;
    }
    case DLCMD_MD_SILENT_PERIOD: // Setting md_silent_period
    {
      if (get_md_silent_period() != downlink_cmd.data_16) {
        set_md_silent_period(downlink_cmd.data_16);
        if (get_md_enable()) {
          lis3dh_md_pause();
          lis3dh_md_resume();
        }
      }
      Serial.printf("Set md silent period = %d\r\n", downlink_cmd.data_16);
      return;
    }
    case DLCMD_REJION: // return to join stage
    {
      f_cmd_rejoin = true;
      return;
    }
    case DLCMD_LINKCHECK_TIMEOUT: // Modify the timeout of link check timeout to return to join stage, range: 0~1440 mintues
    {
      set_linkcheck_timeout((uint32_t)downlink_cmd.data_16);
      reset_linkcheck_start();
      reset_last_send_ok_time();
      Serial.printf("Set link check timeout = %d\r\n", downlink_cmd.data_16);
      return;
    }
  }
}

/*
 * @note: recvCallback, downlink command handler
 */
void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
  uint8_t command;
  uint8_t tmp_8;
  uint16_t tmp_16;
  uint32_t tmp_32;

  if (data->BufferSize > 0) {
    Serial.printf("Received Downlink:");
    for (int i = 0; i < data->BufferSize; i++) {
      Serial.printf(" %02x", data->Buffer[i]);
    }
    Serial.printf("\r\n");

    command = data->Buffer[0] & 0xFF;
    switch(command)
    {
      case DLCMD_DR: // Setting data rate
      {
        if (data->BufferSize != 2)
           Serial.printf("DLCMD_DR(%02X), Parameter error!\r\n", DLCMD_DR);
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if(tmp_8 > 7)
            Serial.printf("DLCMD_DR(%02X), Parameter error!\r\n", DLCMD_DR);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_8 = tmp_8;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_ADR: // Setting adr
      {
        if (data->BufferSize != 2)
           Serial.printf("DLCMD_ADR(%02X) Parameter error\r\n", DLCMD_ADR);
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if (tmp_8 != 0 && tmp_8 != 1)
            Serial.printf("DLCMD_ADR(%02X) Parameter error\r\n", DLCMD_ADR);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_8 = tmp_8;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_UL_INTERVAL: // Modify the interval of periodic uplink, range: 1~1440 mintues
      {
        if (data->BufferSize != 3)
          Serial.printf("DLCMD_UL_INTERVAL(%02X), Parameter error!\r\n", DLCMD_UL_INTERVAL);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 < 1 || tmp_16 > 1440)
            Serial.printf("DLCMD_UL_INTERVAL(%02X), Parameter error!\r\n", DLCMD_UL_INTERVAL);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_ENABLE: // Setting md_enable
      {
        if(data->BufferSize != 2)
           Serial.printf("DLCMD_MD_ENABLE(%02X), Parameter error!\r\n", DLCMD_MD_ENABLE);
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if(tmp_8 > 1)
            Serial.printf("DLCMDLCMD_MD_ENABLED_CFM(%02X), Parameter error!\r\n", DLCMD_MD_ENABLE);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_8 = tmp_8;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_THRESHOLD: // Setting md_threshold
      {
        if (data->BufferSize != 3)
           Serial.printf("DLCMD_MD_THRESHOLD(%02X), Parameter error!\r\n", DLCMD_MD_THRESHOLD);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 < 50 || tmp_16 > 16000)
            Serial.printf("DLCMD_MD_THRESHOLD(%02X), Parameter error!\r\n", DLCMD_MD_THRESHOLD);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_SAMPLE_RATE: // Setting md_sample_rate
      {
        if (data->BufferSize != 3)
           Serial.printf("DLCMD_MD_SAMPLE_RATE(%02X), Parameter error!\r\n", DLCMD_MD_SAMPLE_RATE);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 > 1)
            Serial.printf("DLCMD_MD_SAMPLE_RATE(%02X), Parameter error!\r\n", DLCMD_MD_SAMPLE_RATE);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_CFM: // Enable or disable for sending confirmed package
      {
        if(data->BufferSize != 2)
           Serial.printf("DLCMD_CFM(%02X), Parameter error!\r\n", DLCMD_CFM);
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if(tmp_8 != 0 && tmp_8 != 1)
            Serial.printf("DLCMD_CFM(%02X), Parameter error!\r\n", DLCMD_CFM);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_8 = tmp_8;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_DURATION: // Setting md_duration
      {
        if (data->BufferSize != 3)
           Serial.printf("DLCMD_MD_DURATION(%02X), Parameter error!\r\n", DLCMD_MD_DURATION);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 < 300 || tmp_16 > 15000)
            Serial.printf("DLCMD_MD_DURATION(%02X), Parameter error!\r\n", DLCMD_MD_DURATION);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_NUMBER: // Setting md_number
      {
        if(data->BufferSize != 2)
           Serial.printf("DLCMD_MD_NUMBER(%02X), Parameter error!\r\n", DLCMD_MD_NUMBER);
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if (tmp_8 < 1 || tmp_8 > 10)
            Serial.printf("DLCMD_MD_NUMBER(%02X), Parameter error!\r\n", DLCMD_MD_NUMBER);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_8 = tmp_8;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_MD_SILENT_PERIOD: // Setting md_silent_period
      {
        if(data->BufferSize != 3)
           Serial.printf("DLCMD_MD_SILENT_PERIOD(%02X), Parameter error!\r\n", DLCMD_MD_SILENT_PERIOD);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 < 10 || tmp_16 > 3600)
            Serial.printf("DLCMD_MD_DURATION(%02X), Parameter error!\r\n", DLCMD_MD_DURATION);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      case DLCMD_REJION: // return to join stage
      {
        if (data->BufferSize != 1)
           Serial.printf("DLCMD_REJION(%02X), Parameter error!\r\n", DLCMD_REJION);
        else
        {
          downlink_cmd.length = data->BufferSize;
          downlink_cmd.command = command;

          downlink_ack.length = data->BufferSize;
          memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
          has_downlink_cmd = true;
        }
        break;
      }
      case DLCMD_LINKCHECK_TIMEOUT: // Modify the timeout of link check timeout to return to join stage, range: 0~1440 mintues
      {
        if (data->BufferSize != 3)
          Serial.printf("DLCMD_LINKCHECK_TIMEOUT(%02X), Parameter error!\r\n", DLCMD_LINKCHECK_TIMEOUT);
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if (tmp_16 > 1440)
            Serial.printf("DLCMD_UL_INTERVAL(%02X), Parameter error!\r\n", DLCMD_UL_INTERVAL);
          else {
            downlink_cmd.length = data->BufferSize;
            downlink_cmd.command = command;
            downlink_cmd.data_16 = tmp_16;

            downlink_ack.length = data->BufferSize;
            memcpy(downlink_ack.data, data->Buffer, data->BufferSize);
            has_downlink_cmd = true;
          }
        }
        break;
      }
      default:
      {
        Serial.printf("DLCMD is invalid!\r\n");
      }
    }
  }
}

uint32_t linkcheck_start = 0;

void reset_linkcheck_start()
{
  linkcheck_start = millis();
}

void do_linkcheck()
{
  if (get_linkcheck_timeout() == 0)
    return;

  if ((millis()-linkcheck_start) >= (INTERVAL_LINKCHECK*60*1000))
  {
    service_lora_set_linkcheck(1);
  }
}

/*
 * @note: sending data for periodic uplink
 */
void loraSendData(uint8_t *bufPtr, uint8_t data_len)
{
  if (api.lorawan.njs.get()) 
  {
    memcpy(collected_data , bufPtr  , data_len);
    
    Serial.println("Data Packet:");
    for (int i = 0; i < data_len; i++) 
    {
      Serial.printf("0x%02X ", collected_data[i]);
    }
    Serial.println("");

    do_linkcheck();

    if (api.lorawan.send(data_len, (uint8_t *) & collected_data, FPORT_PERIODIC_UPLINK, api.lorawan.cfm.get(), 1)) 
    {
      Serial.println("Sending is requested");
    } 
    else 
    {
      Serial.println("Sending failed");
    }  
  }
}

void loraSendMDEvent()
{
  if (api.lorawan.njs.get())
  {
    uint8_t data = 0xE0;
    if (api.lorawan.send(1, &data, FPORT_MD_EVENT_UPLINK, api.lorawan.cfm.get(), 1)) 
    {
      Serial.println("MD event sending is requested");
    } 
    else 
    {
      Serial.println("MD event sending failed");
    }
  }
}


/*
 * @note: sendCallback
 */
void sendCallback(int32_t status)
{
  Serial.printf("send_cb(%d)\r\n", status);
  
  if (g_return_pre_setting.check)
  {
    if (api.lorawan.cfs.get() == false)
    {
      g_return_pre_setting.enable = true;
    }
    g_return_pre_setting.check = false;
  }
}


uint8_t lora_network_status = NOT_JOINED;
uint8_t get_lora_network_status(void)
{
  return lora_network_status;
}

void set_lora_network_status(uint8_t status)
{
  lora_network_status = status;
}

uint32_t last_send_ok_time = 0;
void reset_last_send_ok_time(void)
{
  last_send_ok_time = millis();
}

bool get_trigger_rejoin_status(void)
{
  if (get_linkcheck_timeout() == 0)
    return false;

  if ((millis() - last_send_ok_time) >= (get_linkcheck_timeout()*60*1000))
    return true;
  else
    return false;
}


/*
 * @note: linkcheck function callbak for trigger re-join mechanism
 *   re-join after node cannot get linkcheck status over 12 hr
 */
void linkcheckCallback(SERVICE_LORA_LINKCHECK_T *data)
{
  //Serial.printf("linkcheck %d:%d:%d:%d:%d\r\n", data->State, data->DemodMargin, data->NbGateways, data->Rssi, data->Snr);
  if (data->State == 0) {
    set_lora_network_status(SEND_OK);
    reset_last_send_ok_time();
  }
  else {
    set_lora_network_status(SEND_NG);
    if (get_trigger_rejoin_status() == true) {
      set_lora_network_status(NOT_JOINED);
      return_to_join_stage();
    }
  }
}


/*
 * @note: joinCallback, reserve
 */
void joinCallback(int32_t status)
{
  set_lora_network_status(NOT_JOINED);

  Serial.printf("Join status: %d\r\n", status);
  if (status == 0) {
    set_lora_network_status(JOINED);
    reset_last_send_ok_time();

    goto_periodic_uplink_stage();
  }
}


/*
 * @note: initialize LoRaWAN, register callback function
 */
void loraWanInit()
{
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
  service_lora_register_linkcheck_cb(linkcheckCallback);
}
