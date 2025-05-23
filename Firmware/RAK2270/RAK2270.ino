/**
   @file RAK2270.h
   @author rakwireless.com
   @brief For RAK2270.
   @version 1.1.2
   @date 2025-03-05
   @copyright Copyright (c) 2023
**/

#include <Wire.h>
#include <ArduinoJson.h>
#include <CayenneLPP.h>
#include "./custom_at.h"
#include "./ntc.h"
#include "./LoRaWAN.h"
#include "./custom.h"
#include "./lis3dh_md.h"
#include "./RAK2270.h"


CayenneLPP g_solution_data(255);

#define ACT_CHECK   WB_IO1

void factory_mode_setup();
void normal_mode_setup();

void setup()
{
  Serial.begin(115200);
  Serial.printf("RAK2270 SW Version: %s\r\n", CUSTOME_VER);
  delay(500);

  //Initialize ATC command and flash memory
  init_frequency_at();
  init_factory_mode();
  init_ntc_calibration_mode();
  init_development_mode();
  register_rejoin_atcmd();
  init_join_interval();
  init_linkcheck_timeout();
  init_md_atcmd();

  //Initialize NTC, GPIO/output/low
  pinMode(WB_A1, OUTPUT);
  pinMode(WB_A0, OUTPUT);
  digitalWrite(WB_A1, LOW);
  digitalWrite(WB_A0, LOW); // 设置 NTC 采样电路关断  

  uint8_t factoryFlag = get_factory_mode();
  Serial.printf("FACTORY_MODE: %s(%d)\r\n", factoryFlag?"ON":"OFF", factoryFlag);
  get_at_setting(SEND_FREQ_OFFSET);
  Serial.printf("Uplink period is %u minutes\r\n", g_txInterval);

  if (factoryFlag == FACTORY_MODE_ON){
    factory_mode_setup();
  }
  else {
    normal_mode_setup();
  }
}

void factory_mode_setup()
{
  float voltage = api.system.bat.get();
  Serial.printf("Battery Voltage: %2.2f\r\n", voltage);
  float tempT = getTemperature();
  float temperature = calibrateTemperature(tempT);
  Serial.printf("NTC_Temp.: %4.2f\r\n", temperature);
  bool ret = lis3dh_factory_test();
  Serial.printf("LIS3DH: %s\r\n", ret? "OK" : "FAIL");

  pinMode(ACT_CHECK, INPUT);
  udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_NONE);
  if(digitalRead(ACT_CHECK) == HIGH) {
    Serial.println("ACT_CHECK: Inactivated(1)");
  }
  else {
    Serial.println("ACT_CHECK: Activated(0)!!!!!!!!");
  }

  api.lorawan.dcs.set(0);
}

void normal_mode_setup()
{
  lis3dh_init();

  pinMode(ACT_CHECK, INPUT);
  udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_NONE);
  if(digitalRead(ACT_CHECK) == HIGH) {
    Serial.println("ACT_CHECK: Inactivated(1)");
  }
  else {
    Serial.println("ACT_CHECK: Activated(0)!!!!!!!!");
  }

  api.lorawan.dcs.set(1);
  loraWanInit();
}

void factory_mode_loop();
void normal_mode_loop();

void loop()
{
  if (get_factory_mode() == FACTORY_MODE_ON) { //FACTORY_MODE
    factory_mode_loop();
  }
  else {
    normal_mode_loop();
  }
}

void factory_mode_loop()
{
  if (get_ntc_calibration_mode() && (api.lorawan.nwm.get()==1))
  {
    if (api.lorawan.njs.get() == 0)
    {
      Serial.print("Waiting for Lorawan join...\r\n");
      api.lorawan.join(1,0,7,0);
      delay(10000);
    }
    else
    {
      periodic_uplink_handler();
      delay(30000);
    }
  }
}

uint8_t g_system_stage = ACTIVATE_STAGE;

uint8_t get_system_stage()
{
  return g_system_stage;
}

bool activate_debounce_check()
{
  uint32_t timeout = ACTIVATE_DEBOUNCE_TIMEOUT;
  uint32_t start = millis();
  do
  {
    api.system.sleep.all(500);
    if(digitalRead(ACT_CHECK) == HIGH)
      return false;
  } while ((millis()-start) < timeout);
  return true;
}

void downlink_check_handle()
{
  if (g_return_pre_setting.enable) {
    if (g_return_pre_setting.cmd == DLCMD_DR) {
      api.lorawan.dr.set(g_return_pre_setting.data_8);
      Serial.printf("Return DR to %d\r\n", g_return_pre_setting.data_8);
    }
    else if (g_return_pre_setting.cmd == DLCMD_ADR) {
      api.lorawan.adr.set(g_return_pre_setting.data_8);
      Serial.printf("Return ADR to %d\r\n", g_return_pre_setting.data_8);
    }
    g_return_pre_setting.enable = false;
  }
}

void normal_mode_loop()
{
  switch (g_system_stage) {
    case ACTIVATE_STAGE:
      if (get_development_mode())
      {
        Serial.println("Device activated for development.");
        goto_join_stage();
      }

      if(digitalRead(ACT_CHECK) == HIGH)
      {
        api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, ACT_CHECK);
        api.system.sleep.all();
        udrv_gpio_set_wakeup_disable(ACT_CHECK);

        if (activate_debounce_check())
        {
          Serial.println("Device activated.");
          api.system.reboot();
          //goto_join_stage();
        }
      }
      else
      {
        if (activate_debounce_check())
        {
          Serial.println("Device activated.");
          goto_join_stage();
        }
      }
      break;
    case JOIN_STAGE:
      api.system.sleep.all();
      break;
    case PERIODIC_UPLINK_STAGE:
      api.system.sleep.all();

      //downlink command handle
      if (has_downlink_cmd)
      {
        downlink_cmd_handle();

        if (f_cmd_rejoin)
        {
          f_cmd_rejoin = false;
          return_to_join_stage();
        }

        if (g_return_pre_setting.check)
        {
          api.system.timer.stop(TIMER_DOWNLINK_CHECK);
          if (api.system.timer.create(TIMER_DOWNLINK_CHECK, (RAK_TIMER_HANDLER)downlink_check_handle, RAK_TIMER_ONESHOT) != true)
          {
            Serial.printf("Creating timer failed.\r\n");
          } 
          else if (api.system.timer.start(TIMER_DOWNLINK_CHECK, 5000, NULL) != true)
          {
            Serial.printf("Starting timer failed.\r\n");
          }
        }
      }
      break;
  }
}

void goto_join_stage()
{
  Serial.println("Goto join stage.");

  //create timer for join
  api.system.timer.stop(TIMER_PERIODIC_UPLINK);
  if (api.system.timer.create(TIMER_PERIODIC_UPLINK, (RAK_TIMER_HANDLER)join_handler, RAK_TIMER_PERIODIC) != true)
  {
    Serial.printf("Creating timer failed.\r\n");
  } 
  else if (api.system.timer.start(TIMER_PERIODIC_UPLINK, get_join_interval() *60 * 1000, NULL) != true)
  {
    Serial.printf("Starting timer failed.\r\n");
  }

  join_handler();

  g_system_stage = JOIN_STAGE;
}

void return_to_join_stage()
{
  if (get_md_enable())
  {
    lis3dh_md_end();
  }
  goto_join_stage();
}

void goto_periodic_uplink_stage()
{
  Serial.println("Goto periodic uplink stage.");

  //create timer for periodic uplink
  api.system.timer.stop(TIMER_PERIODIC_UPLINK);
  if (api.system.timer.create(TIMER_PERIODIC_UPLINK, (RAK_TIMER_HANDLER)periodic_uplink_handler, RAK_TIMER_PERIODIC) != true)
  {
    Serial.printf("Creating timer failed.\r\n");
  } 
  else if (api.system.timer.start(TIMER_PERIODIC_UPLINK, g_txInterval * 60 * 1000, NULL) != true)
  {
    Serial.printf("Starting timer failed.\r\n");
  }

  reset_linkcheck_start();
  reset_last_send_ok_time();
  periodic_uplink_handler();

  //start lis3dh for motion detection
  if (get_md_enable())
  {
    lis3dh_md_begin();
  }

  g_system_stage = PERIODIC_UPLINK_STAGE;
}

void inactivate_protect_check()
{
  if (get_development_mode())
  {
    return;
  }

  if (digitalRead(ACT_CHECK) == HIGH)
  {
    Serial.println("activate_protect_check: Inactivated!!!!");
    api.system.reboot();
  }
}

void join_handler(void)
{
  inactivate_protect_check();

  //join lorawan network immediately
  Serial.println("Wait for LoRaWAN join...\r\n");
  if (!api.lorawan.join(1,0,7,3))  // Join to Gateway
  {
    Serial.printf("LoRaWan OTAA - join fail! \r\n");
  }
}

void periodic_uplink_handler(void)
{
  inactivate_protect_check();

  Serial.println("\r\nperiodic_uplink_handler");
  float temperature = 0;
  float voltage = 0;
  float tempT = getTemperature();

  temperature = calibrateTemperature(tempT);
  voltage     = api.system.bat.get();
  Serial.printf("temperature = %4.2f\r\n", temperature);
  Serial.printf("voltage = %2.2f\r\n", voltage);

  g_solution_data.reset();
  g_solution_data.addTemperature(TEMP_CH, temperature);
  g_solution_data.addAnalogInput(BAT_CH, voltage);

  loraSendData(g_solution_data.getBuffer(), g_solution_data.getSize());
}
