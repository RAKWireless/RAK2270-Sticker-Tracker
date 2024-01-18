/**
   @file RAK2270.h
   @author rakwireless.com
   @brief For RAK2270.
   @version 1.0.0
   @date 2022-10-3
   @copyright Copyright (c) 2023
**/

#include <Wire.h>
#include <ArduinoJson.h>
#include <CayenneLPP.h>
#include "./custom_at.h"
#include "./ntc.h"
#include "./LoRaWAN.h"
#include "./custom.h"
#include "./RAK2270.h"


CayenneLPP g_solution_data(255);
static uint8_t factoryFlag = DEFAULT_FACTORY_MODE;

#define ACT_CHECK   WB_IO1

void factory_mode_setup();
void normal_mode_setup();

void setup()
{
  Serial.begin(115200);
  Serial.printf("RAK2270 SW Version: %s\r\n", CUSTOME_VER);
  delay(500);

  //Initialize ATC command and flash memory
  init_factory_mode();
  init_frequency_at();

  //Initialize NTC, GPIO/output/low
  pinMode(WB_A1, OUTPUT);
  pinMode(WB_A0, OUTPUT);
  digitalWrite(WB_A1, LOW);
  digitalWrite(WB_A0, LOW); // 设置 NTC 采样电路关断  

  factoryFlag = get_factory_mode();
  Serial.printf("FACTORY_MODE: %s(%d)\r\n", factoryFlag?"ON":"OFF", factoryFlag);
  get_at_setting(SEND_FREQ_OFFSET);
  Serial.printf("Uplink period is %u minutes\r\n", g_txInterval);

  if (factoryFlag != FACTORY_MODE_OFF){
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

  pinMode(ACT_CHECK, INPUT);
  udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_NONE);
  if(digitalRead(ACT_CHECK) == HIGH) {
    Serial.println("ACT_CHECK: Inactivated(1)");
  }
  else {
    Serial.println("ACT_CHECK: Activated(0)!!!!!!!!");
  }

  if (api.lorawan.nwm.get()) { // LoRaWAN mode for temperature test
    loraWanInit();
  }
}

void normal_mode_setup()
{
  uint8_t production_delay_flag = 0;
  loraWanInit();

  pinMode(ACT_CHECK, INPUT);
  udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_NONE);
  if(digitalRead(ACT_CHECK) == HIGH) // LOW
  {
    Serial.println("Device inactivated.");
    api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, ACT_CHECK); //  RUI_WAKEUP_RISING_EDGE
    api.system.sleep.all();

    Serial.println("Device activated.");
    udrv_gpio_set_wakeup_disable(ACT_CHECK);
    delay(500);
    if(digitalRead(ACT_CHECK) == LOW) {
      udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_DOWN);
    }
    else { //for production test
      production_delay_flag = 1;
    }
  }
  else
  {
    Serial.println("Device activated.");
    udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_DOWN);
  }

  //delay for production test
  if (production_delay_flag) {
    Serial.println("Delay 10 sec for production test");
    delay(10000);
  }

  if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)period_handler, RAK_TIMER_PERIODIC) != true)
  {
    Serial.printf("Creating timer failed.\r\n");
  } 
  else if (api.system.timer.start(RAK_TIMER_0, g_txInterval * 60 * 1000, NULL) != true)
  {
    Serial.printf("Starting timer failed.\r\n");
  }

  //join lora
  Serial.println("Wait for LoRaWAN join...");
  if (!api.lorawan.join(1,0,7,3))  // Join to Gateway
  {
    Serial.printf("LoRaWan OTAA - join fail! \r\n");
  }
}


void factory_mode_loop();

void loop()
{
  if (factoryFlag == FACTORY_MODE_OFF) {
#if ENABLE_FACTORY_MODE_SLEEP
    api.system.sleep.all();
#else
    api.system.scheduler.task.destroy();
#endif
  }
  else { //FACTORY_MODE
#if ENABLE_TEMPERTURE_TEST
    factory_mode_loop();
#else
    api.system.scheduler.task.destroy();
#endif
  }
}

void factory_mode_loop()
{ 
  if (api.lorawan.nwm.get()) { // LoRaWAN mode for Temperature test
    uint8_t data[9] = {0};
    short data_value = 0;
    delay(30000);

    //voltage
    float voltage = api.system.bat.get();
    Serial.printf("Battery Voltage: %2.2f\r\n", voltage);
    data_value = (short)(voltage * 100);
    data[0] = 0x1; //channel
    data[1] = 0xBA; //IPSO
    data[2] = (data_value>>8)&0xff;
    data[3] = data_value&0xff;

    float tempT = getTemperature();
    float temperature = calibrateTemperature(tempT);
    Serial.printf("NTC_Temp.: %4.2f\r\n", temperature);
    data_value = (short)(temperature * 10);
    data[4] = 0x3; //channel
    data[5] = 0x67; //IPSO
    data[6] = (data_value>>8)&0xff;
    data[7] = data_value&0xff;

    loraSendTempDate(8, data);
  }
  else { // P2P mode, for RF test
    api.system.scheduler.task.destroy();
  }
}


void period_handler(void)
{
  Serial.println("\r\nperiod_handler");
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

  loraSendDate(g_solution_data.getBuffer(), g_solution_data.getSize());
}
