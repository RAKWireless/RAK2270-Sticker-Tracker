#include "service_lora.h"
#include "udrv_flash.h"
#include "./custom_at.h"
#include "./LoRaWAN.h"
#include "./RAK2270.h"


#define DOWNLINK_ACK_INTERVAL   (10000)

static uint16_t interval = INTERVAL_PERIODIC_UPLINK;
static bool confirm = LORAWAN_CONFIRMED;
static uint8_t retry = LORAWAN_RETRY;
static uint8_t fport = FPORT_PERIODIC_UPLINK;

/** Packet buffer for sending */
uint8_t collected_data[64] = { 0 };

typedef struct DOWNLINK_ACK_s
{
  uint8_t length;
  uint8_t data;
} DOWNLINK_ACK_t; // 16bytes

DOWNLINK_ACK_t downlink_ack;

typedef enum
{
  NOT_JOINED = 0x0,
  JOINED = 0x1,
  SEND_OK = 0x2,
  SEND_NG = 0x3,
} LORA_NETWORK_STAUS_t;

uint8_t get_lora_network_status(void);

void send_downlink_ack(void)
{
  if (api.lorawan.send(downlink_ack.length, &downlink_ack.data, fport, api.lorawan.cfm.get(), retry))
    Serial.println("Sending is requested");
  else
    Serial.println("Sending failed");
}

/*
 * @note: recvCallback, downlink command handler
 */
void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
  uint8_t tmp_8;
  uint16_t tmp_16;
  uint32_t tmp_32;
  uint8_t command;
  if (data->BufferSize > 0) {
    Serial.printf("Something received:");
    for (int i = 0; i < data->BufferSize; i++) {
      Serial.printf(" %02x", data->Buffer[i]);
    }
    Serial.printf("\r\n");

    command = data->Buffer[0] & 0xFF;
    switch(command)
    {
      case 0: //Reserve
      {
              break;
      }
      case 1: // Reserve
      {
              break;
      }
      case 2: // Modify the interval of periodic uplink, range: 1~1440 mintues
      {
        if(data->BufferSize != 3)
                    Serial.printf("command 0x02 error\r\n");
        else
        {
          tmp_16 = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
          if(0x001 > tmp_16 || 0x05A0 < tmp_16)
            Serial.printf("command 0x02 parameter error\r\n");
          else
            interval = tmp_16;
          api.system.timer.stop(RAK_TIMER_0);
          if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)period_handler, RAK_TIMER_PERIODIC) != true)
            Serial.printf("Creating timer failed.\r\n");
          else if (api.system.timer.start(RAK_TIMER_0, interval * 60000, NULL) != true)
            Serial.printf("Starting timer failed.\r\n");

          if (api.lorawan.send(data->BufferSize, (uint8_t *) data->Buffer, fport, api.lorawan.cfm.get(), retry))
            Serial.println("Sending is requested");
          else
            Serial.println("Sending failed");
          g_txInterval = interval;
          Serial.printf("Set period uplink interval = %d\r\n",g_txInterval);
          save_at_setting(SEND_FREQ_OFFSET);
        }
        break;
      }
      case 3: // Reserve
      {
        break;
      }
      case 4: // Reserve
      {
        break;
      }
      case 5: // Reserve
      {
        break;
      }
      case 6: // Enable or disable for sending confirmed package
      {
        if(data->BufferSize != 2)
           Serial.printf("command 0x06 error\r\n");
        else
        {
          tmp_8 = (data->Buffer[1] & 0xFF);
          if(tmp_8 != 0 && tmp_8 != 1)
            Serial.printf("command 0x03 parameter error\r\n");
          else {
            if(data->Buffer[1] == 0x00) {
              if (!api.lorawan.cfm.set(0)) {
                Serial.printf("LoRaWan set confirm mode is incorrect! \r\n");
                return;
              }
              else {
                Serial.printf("Set CFM = 0\r\n");
              }
            }
            else if(data->Buffer[1] == 0x01) {
              if (!api.lorawan.cfm.set(1)) {
                Serial.printf("LoRaWan set confirm mode is incorrect! \r\n");
                return;
              }
              else {
                Serial.printf("Set CFM = 1\r\n");
              }
            }
          }

          api.system.timer.stop(RAK_TIMER_1);
          memset(&downlink_ack, 0, sizeof(DOWNLINK_ACK_t));
          downlink_ack.length = data->BufferSize;
          memcpy(&downlink_ack.data, (uint8_t *)data->Buffer, data->BufferSize);
          if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)send_downlink_ack, RAK_TIMER_ONESHOT) != true) 
            Serial.printf("Creating timer failed.\r\n");
          else if (api.system.timer.start(RAK_TIMER_1, DOWNLINK_ACK_INTERVAL, NULL) != true) 
            Serial.printf("Starting timer failed.\r\n");
        }
        break;
      }
      default:
        Serial.printf("Invalid command!\r\n");
    }
  }
}


/*
 * @note: sending data for periodic uplink
 */
void loraSendDate(uint8_t *bufPtr, uint8_t data_len)
{
  if (api.lorawan.njs.get() == 0  || get_lora_network_status() == NOT_JOINED) 
  {
    Serial.println("Wait for LoRaWAN join...");
    if (!api.lorawan.join(1,0,7,3))  // Join to Gateway
    {
      Serial.printf("LoRaWan OTAA - join fail! \r\n");
    }
    return;
  }
  else
  {
    memcpy(collected_data , bufPtr  , data_len);
    
    Serial.println("Data Packet:");
    for (int i = 0; i < data_len; i++) 
    {
      Serial.printf("0x%02X ", collected_data[i]);
    }
    Serial.println("");

    if (api.lorawan.send(data_len, (uint8_t *) & collected_data, fport, api.lorawan.cfm.get(), 1)) 
    {
      Serial.println("Sending is requested");
    } 
    else 
    {
      Serial.println("Sending failed");
    }  
  }
}

/*
 * @note: Sending temperature data for test, only support in factory mode
 */
void loraSendTempDate(uint8_t len, uint8_t *buf)
{
  
  if (api.lorawan.njs.get() == 0) 
  {
    Serial.println("Wait for LoRaWAN join...");
    if (!api.lorawan.join(1,0,7,3)) // join attempt, 1+3 retry
    {
      Serial.printf("LoRaWan OTAA - join fail! \r\n");
    }
    return;
  }
  else
  {
    memcpy(collected_data, buf, len);
    
    Serial.println("Data Packet:");
    for (int i = 0; i < len; i++) 
    {
      Serial.printf("0x%02X ", collected_data[i]);
    }
    Serial.println("");
    
    if (api.lorawan.send(len, (uint8_t *) & collected_data, fport, api.lorawan.cfm.get(), 1)) 
    {
      Serial.println("Sending is requested");
    } 
    else 
    {
      Serial.println("Sending failed");
    }  
  }
}


/*
 * @note: sendCallback, reserve
 */
void sendCallback(int32_t status)
{

}


#define DEFAULT_TRIGGER_REJION_INTERVERAL   (12*60*60*1000) //12hr, 12*60*60*1000
#define DEFAULT_LINKCHECK_INTERVAL          (1*60*60*1000)  //1hr, 1*60*60*1000


uint8_t lora_network_status = NOT_JOINED;
uint8_t get_lora_network_status(void)
{
  return lora_network_status;
}

void set_lora_network_status(uint8_t status)
{
  lora_network_status = status;
}

uint64_t last_send_ok_time = 0;
void reset_last_send_ok_time(void)
{
  last_send_ok_time = millis();
}

uint64_t trigger_rejoin_interval = DEFAULT_TRIGGER_REJION_INTERVERAL; //12 hr
void set_trigger_rejoin_interval(uint64_t interval)
{
  trigger_rejoin_interval = interval;
}

bool get_trigger_rejoin_status(void)
{
  uint64_t current = millis();
  if ((current - last_send_ok_time) >= trigger_rejoin_interval)
    return true;
  else
    return false;
}

void linkcheck_handler(void)
{
  service_lora_set_linkcheck(1);
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
    }
  }
}


/*
 * @note: joinCallback, reserve
 */
void joinCallback(int32_t status)
{
  Serial.printf("Join status: %d\r\n", status);
  if (status == 0) {
    set_lora_network_status(JOINED);
    reset_last_send_ok_time();

    api.system.timer.stop(RAK_TIMER_3);
    if (api.system.timer.create(RAK_TIMER_3, (RAK_TIMER_HANDLER)linkcheck_handler, RAK_TIMER_PERIODIC) != true)
      Serial.printf("Creating timer failed.\r\n");
    else if (api.system.timer.start(RAK_TIMER_3, DEFAULT_LINKCHECK_INTERVAL, NULL) != true)
      Serial.printf("Starting timer failed.\r\n");

    api.system.timer.stop(RAK_TIMER_0);
    if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)period_handler, RAK_TIMER_PERIODIC) != true)
      Serial.printf("Creating timer failed.\r\n");
    else if (api.system.timer.start(RAK_TIMER_0, g_txInterval * 60 * 1000, NULL) != true)
      Serial.printf("Starting timer failed.\r\n");

    period_handler();
  }
  else {
    set_lora_network_status(NOT_JOINED);
  }
}


/*
 * @note: initialize LoRaWAN, register callback function
 */
void loraWanInit()
{
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  //api.lorawan.registerSendCallback(sendCallback);
  service_lora_register_linkcheck_cb(linkcheckCallback);
}
