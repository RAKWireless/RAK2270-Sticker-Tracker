/**
   @file RAK2270.ino
**/

#define TEMP_CH     0
#define BAT_CH      0

#define MAJOR_VERSION  0x01
#define MINOR_VERSION  0x01

#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
#include <Wire.h>
#include <ArduinoJson.h>
#include <CayenneLPP.h>
#include "custom_at.h"
#include "udrv_flash.h"
#include "service_custom.h"
#include "ntc.h"
#include "bl24c.h"
#include "LoRaWAN.h"

#define FW_VERSION  "v1.0.0.RC6"

typedef enum
{
  CUSTOM_TEST = 0x0,
  CUSTOM_CERTIFICATION = 0x1,
  CUSTOM_STANDARD = 0x02,
  CUSTOM_NOVALABS = 0x3,
} CUSTOM_ID_t;
const uint8_t custom_id = CUSTOM_STANDARD;

#define ENABLE_TEMPERTURE_TEST (0)

stored_data cached_buf[256];
uint16_t cached_buf_first = 0;
uint16_t cached_buf_last = 0;

/*
 * @note: for存储
 */
uint32_t start_time = 0;
uint32_t end_time = 0;
uint16_t start_seq_no = 0;
uint16_t end_seq_no = 0;
uint32_t start_time_bin = 0;
uint32_t end_time_bin = 0;
uint16_t start_seq_no_bin = 0;
uint16_t end_seq_no_bin = 0;
uint16_t interval = 0;
uint8_t is_sending = 0;
bool confirm = false;
uint8_t retry = 0;
uint8_t fport = 10;
uint8_t fport_period_uplink = 2;
uint8_t fport_event_uplink = 3;
uint8_t fport_retrieval_uplink = 4;
uint8_t data_len = sizeof(stored_data);
uint16_t maskBuff = 0x0003;
uint16_t flash_buf[USER_BUFFER_SIZE];
uint16_t flash_buf_pos = 0;

LIS3DH myIMU(I2C_MODE, 0x19);
#define LIS3D_INT1    WB_IO7
#define LIS3D_INT2    PB_14
volatile uint8_t g_int1Flag = 0;
volatile uint8_t g_int2Flag = 0;
volatile uint8_t g_motionLevel = NO_MOVE;
volatile uint8_t g_shockLevel  = NO_SHOCK;
volatile uint16_t g_shockCount  = 0;
workMode_t workMode = MOTION_DETECTION_MODE; // MOTION_DETECTION_MODE( default ) , SHOCK_DETECTION_MODE


#define ACT_CHECK   WB_IO1

volatile uint8_t accIntFlag = false;
static uint8_t factoryFlag = 0xFF;

CayenneLPP g_solution_data(255);


static uint64_t last      = 0;
static uint64_t elapsed;
static float    temp      = 0;
static float    voltage   = 0;

void period_handler(void);
void accIntHandle1(void);
void accIntHandle2(void);

void WakeupCallback()
{
  Serial.printf("This is Wakeup Callback\r\n");
}

//#define DEBUG_LIS3DH
#ifdef DEBUG_LIS3DH
void showIMUReg(void)
{
  uint8_t readData = 0;
  myIMU.readRegister(&readData, LIS3DH_STATUS_REG_AUX);
  Serial.printf("LIS3DH_STATUS_REG_AUX: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT_COUNTER_REG);
  Serial.printf("LIS3DH_INT_COUNTER_REG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_TEMP_CFG_REG);
  Serial.printf("LIS3DH_TEMP_CFG_REG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG1);
  Serial.printf("LIS3DH_CTRL_REG1: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG2);
  Serial.printf("LIS3DH_CTRL_REG2: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG3);
  Serial.printf("LIS3DH_CTRL_REG3: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG4);
  Serial.printf("LIS3DH_CTRL_REG4: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG5);
  Serial.printf("LIS3DH_CTRL_REG5: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CTRL_REG6);
  Serial.printf("LIS3DH_CTRL_REG6: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_STATUS_REG2);
  Serial.printf("LIS3DH_STATUS_REG2: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_FIFO_CTRL_REG);
  Serial.printf("LIS3DH_FIFO_CTRL_REG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  Serial.printf("LIS3DH_INT1_CFG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_SRC);
  Serial.printf("LIS3DH_INT1_SRC: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_THS);
  Serial.printf("LIS3DH_INT1_THS: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_DURATION);
  Serial.printf("LIS3DH_INT1_DURATION: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);
  Serial.printf("LIS3DH_INT2_CFG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_SRC);
  Serial.printf("LIS3DH_INT2_SRC: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_THS);
  Serial.printf("LIS3DH_INT2_THS: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_DURATION);
  Serial.printf("LIS3DH_INT2_DURATION: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_CLICK_CFG);
  Serial.printf("LIS3DH_CLICK_CFG: 0x%02X\r\n", readData);
}
#endif


#define BL24C_WP    PB13
bool check_BL24C(void)
{
  bool testResult = false;

  pinMode(BL24C_WP, OUTPUT);
  digitalWrite(BL24C_WP, LOW);

  //Wire.begin();
  delay(100);
  testResult = testBL24C();

//  pinMode(BL24C_WP, INPUT);
//  udrv_gpio_set_pull((uint32_t)BL24C_WP, GPIO_PULL_NONE);

  return testResult;
}

void setup()
{
  Serial.begin(115200);
  if (custom_id == CUSTOM_STANDARD) {
    Serial.printf("RAK2270 SW Version: %s.ForStandard\r\n", FW_VERSION);
  }
  else if (custom_id == CUSTOM_NOVALABS) {
    Serial.printf("RAK2270 SW Version: %s.ForNovalabs\r\n", FW_VERSION);
  }
  delay(500);

  //Initialize ATC command and flash memory
  init_factory_mode();
  register_factory_mode_atcmd();
  init_frequency_at();

  //Initialize NTC, GPIO/output/low
  pinMode(WB_A1, OUTPUT);
  pinMode(WB_A0, OUTPUT);
  digitalWrite(WB_A1, LOW);
  digitalWrite(WB_A0, LOW); // 设置 NTC 采样电路关断  

  //Initialize ACC module, GPIO/input/high
  pinMode(WB_I2C1_SDA, INPUT_PULLUP);
  pinMode(WB_I2C1_SCL, INPUT_PULLUP); // 设置 ACC IIC 电路

  //Initialize interrupt for ACC module, GPIO/input/low
  pinMode(PA10, INPUT_PULLDOWN);
  pinMode(PB14, INPUT_PULLDOWN);  // 设置 ACC INT 关断

  factoryFlag = get_factory_mode();
  //factoryFlag = FACTORY_MODE_OFF; // for test.
  Serial.printf("FACTORY_MODE: %s(%d)\r\n", factoryFlag?"ON":"OFF", factoryFlag);

  get_at_setting(SEND_FREQ_OFFSET);
  Serial.printf("Uplink period is %u minutes\r\n", g_txInterval);

  if (factoryFlag != FACTORY_MODE_OFF){ //FACTORY_MODE
    float voltage = api.system.bat.get();
    Serial.printf("Battery Voltage: %2.2f\r\n", voltage);
    float tempT = getTemperature();
    float temperature = (0.00312f * tempT * tempT) + (0.927f * tempT) - 1.2f;
    Serial.printf("NTC_Temp.: %4.2f\r\n", temperature);

    uint8_t readData = 0;
    myIMU.settings.adcEnabled = 0;
    myIMU.settings.tempEnabled = 0;
    myIMU.settings.xAccelEnabled = 0;
    myIMU.settings.yAccelEnabled = 0;
    myIMU.settings.zAccelEnabled = 0;

    if (myIMU.begin()!= 0) {
      //Serial.printf("Init LIS3DH fail...\r\n");
      Serial.printf("LIS3DH_WHO_AM_I: 0x%02X\r\n", readData);
    }
    else {
      myIMU.readRegister(&readData, LIS3DH_WHO_AM_I);
      //showIMUReg();
      Serial.printf("LIS3DH_WHO_AM_I: 0x%02X\r\n", readData);
    }

    //check EEPROM
    if (check_BL24C() == true) {
      Serial.println("BL24C_CHECK: OK");
    }
    else {
      Serial.println("BL24C_CHECK: NG");
    }

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
  else { //NORMAL MODE
    uint8_t production_delay_flag = 0;
    loraWanInit();

    pinMode(ACT_CHECK, INPUT);
    udrv_gpio_set_pull((uint32_t)ACT_CHECK, GPIO_PULL_NONE);
    if(digitalRead(ACT_CHECK) == HIGH) // LOW
    {
      Serial.println("Device inactivated.");
      api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, ACT_CHECK); //  RUI_WAKEUP_RISING_EDGE
      ///if ( api.system.sleep.registerWakeupCallback(WakeupCallback) == false )
      //{
      //  Serial.println("Create Wakeup Callback failed.");
      //}
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
      Serial.println("Delay for production test");
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

    if (custom_id == CUSTOM_NOVALABS) {
      init_event_mode();
      uint8_t event_mode = get_event_mode();
      if (event_mode & EVENT_MOTION_DETECTION) {
        Serial.printf("Motion detection enable\r\n");
        Serial.printf("Shock detection disable\r\n");
        workMode = MOTION_DETECTION_MODE;
      }
      else if (event_mode & EVENT_SHOCK_DETECTION) {
        Serial.printf("Motion detection disable\r\n");
        Serial.printf("Shock detection enable\r\n");
        workMode = SHOCK_DETECTION_MODE;
      }
      else {
        Serial.printf("Motion detection disable\r\n");
        Serial.printf("Shock detection disable\r\n");
        //workMode = NONE_MODE;
        workMode = MOTION_DETECTION_MODE;
      }
      myIMU.begin(LIS3D_INT1,LIS3D_INT2,accIntHandle2,accIntHandle1);
      myIMU.setMode(workMode);
      if (!((event_mode & EVENT_MOTION_DETECTION)||(event_mode & EVENT_SHOCK_DETECTION))) {
        detachInterrupt(LIS3D_INT1);
        detachInterrupt(LIS3D_INT2);
      }
      //showIMUReg();
    }

    //join lora
    Serial.println("Wait for LoRaWAN join...");
    if (!api.lorawan.join(1,0,7,3))  // Join to Gateway
    {
      Serial.printf("LoRaWan OTAA - join fail! \r\n");
    }

  } //NORMAL MODE
}


void factory_mode_loop();
uint8_t myIMU_lock = 0;

#define ENABLE_SLEEP_MODE (1)
void loop()
{
  if (factoryFlag == FACTORY_MODE_OFF) { //NORMAL MODE
#if ENABLE_SLEEP_MODE
    //myIMU_lock
    if (myIMU_lock == 0) {
      //Serial.printf("loop sleep\r\n");
      api.system.sleep.all();
    }
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
    voltage = api.system.bat.get();
    Serial.printf("Battery Voltage: %2.2f\r\n", voltage);
    data_value = (short)(voltage * 100);
    data[0] = 0x1; //channel
    data[1] = 0xBA; //IPSO
    data[2] = (data_value>>8)&0xff;
    data[3] = data_value&0xff;

    //temp = getTemperature() - 1.5;
    float tempT = getTemperature();
    float temperature = (0.00312f * tempT * tempT) + (0.927f * tempT) - 1.2f;
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


void deinitPeripheral()
{
  pinMode(WB_A1, OUTPUT);
  pinMode(WB_A0, OUTPUT);
  digitalWrite(WB_A1, LOW);
  digitalWrite(WB_A0, LOW); // 设置 NTC 采样电路关断  

  pinMode(WB_I2C1_SDA, INPUT_PULLUP);
  pinMode(WB_I2C1_SCL, INPUT_PULLUP); // 设置 ACC IIC 电路

  pinMode(PA10, INPUT_PULLDOWN);
  pinMode(PB14, INPUT_PULLDOWN);  // 设置 ACC INT 关断

  //myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x08); // 设置 ACC 进入 LowerPowerMOde.
}

#define USE_FAKE_DATA (0)
void period_handler(void)
{
  Serial.println("\r\nperiod_handler");
  static uint16_t seqNum = 0;
  stored_metadata metadata;
  stored_data read_data;
  float temperature = 0;
  float voltage = 0;

#if USE_FAKE_DATA
  static uint8_t  fakeDate = 0;
  fakeDate++;
  temperature = fakeDate;
  voltage = fakeDate;
#else
  float tempT = getTemperature();
  temperature = (0.00312f * tempT * tempT) + (0.927f * tempT) - 1.2f;
  //temperature = getTemperature();
  //temperature = temperature - 1.5;
  voltage     = api.system.bat.get();
#endif
  Serial.printf("temperature = %4.2f\r\n", temperature);
  Serial.printf("voltage = %2.2f\r\n", voltage);

  if (custom_id == CUSTOM_STANDARD) { //CUSTOM_STANDARD

    g_solution_data.reset();
    g_solution_data.addTemperature(TEMP_CH, temperature);
    g_solution_data.addAnalogInput(BAT_CH, voltage);
      
    loraSendDate();

  } //CUSTOM_STANDARD
  else if (custom_id == CUSTOM_NOVALABS) { //CUSTOM_NOVALABS

  if (api.lorawan.njs.get() == 0) // 若没入网，则进行5次 入网尝试，入网尝试间隔为8s。
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
    service_custom_read_user_metadata(&metadata);
    if (metadata.write_pos == 0xFFFF)
      metadata.write_pos = 0;
    seqNum = metadata.write_pos + 1;
    Serial.printf("seqNum = %d\r\n", seqNum);

    if(seqNum > USER_BUFFER_SIZE)
      seqNum = 0;
    
    uint8_t collected_data[20] = { 0 };
    uint8_t data_len = 0;
    
    read_data.seq_no    = (((uint16_t)(MAJOR_VERSION<<14)) | ((uint16_t)(MINOR_VERSION<<12)) | seqNum);
    read_data.payload   = (uint16_t)(temperature * 10);
    read_data.timestamp = millis()/1000; // 计数会在 第 48 天复位。
    //read_data.vbat      = (uint16_t)(voltage*100);

    service_custom_write_user_data(&read_data , 1 , false); // 将数据写入存储

    collected_data[data_len++] = (uint8_t)(read_data.seq_no >> 8);
    collected_data[data_len++] = (uint8_t)read_data.seq_no;
    collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0xFF000000) >> 24);;
    collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0x00FF0000) >> 16);
    collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0x0000FF00) >> 8);
    collected_data[data_len++] = (uint8_t)(read_data.timestamp & 0x000000FF);
    collected_data[data_len++] = (uint8_t)(read_data.payload >> 8);
    collected_data[data_len++] = (uint8_t)read_data.payload;


    if ((elapsed = millis() - last) > 20000) {
      if (api.lorawan.send(data_len, (uint8_t *) & collected_data, fport_period_uplink, api.lorawan.cfm.get(), 1)) // 周期发送数据
        Serial.println("Sending is requested");
      else
        Serial.println("Sending failed");
      last = millis();
    }

    Serial.printf("[SEND] date len:%d date:",data_len);
    for(int i = 0; i < data_len; i++)
    {
      Serial.printf("0x%02x,",collected_data[i]); 
    }
    Serial.println();

    //use timer handle to send event uplink
    if ( ((get_event_mode() & EVENT_MOTION_DETECTION)&&(g_motionLevel != NO_MOVE)) || ((get_event_mode() & EVENT_SHOCK_DETECTION)&&(g_shockLevel != NO_SHOCK)) ) {
      if (api.system.timer.create(RAK_TIMER_2, (RAK_TIMER_HANDLER)send_event_data, RAK_TIMER_ONESHOT) != true) 
        Serial.printf("Creating timer failed.\r\n");
      else if (api.system.timer.start(RAK_TIMER_2, RETRIEVAL_UPLINK_INTERVAL, NULL) != true) 
        Serial.printf("Starting timer failed.\r\n");
    }
  }

  } //CUSTOM_NOVALABS
}

void send_event_data(void )
{
    uint8_t collected_data[20] = { 0 };
    uint8_t data_len = 0;

    data_len = 0;
    memset(collected_data, 0, sizeof(collected_data));    

    if((workMode == MOTION_DETECTION_MODE)&&(g_motionLevel != NO_MOVE)) // // 发送事件数据
    {
      collected_data[data_len++] = 0x00;  // Motion detection
      if(g_motionLevel == MOTION_LOW)
        collected_data[data_len++] = 0x00; // MOTIN level LOW
      else if(g_motionLevel == MOTION_HIGH)
        collected_data[data_len++] = 0x01; // MOTIN level HIGH
      collected_data[data_len++] = 0x00;// RFU
      collected_data[data_len++] = 0x00;// RFU

      Serial.printf("[SEND] date len:%d date:",data_len);
      for(int i = 0; i < data_len; i++)
      {
        Serial.printf("0x%02x,",collected_data[i]); 
      }
      Serial.println();

      if (api.lorawan.send(data_len, (uint8_t *) & collected_data, fport_event_uplink, api.lorawan.cfm.get(), 1))
          Serial.println("Sending is requested");
      else
          Serial.println("Sending failed");
          
      g_motionLevel = NO_MOVE;
    }
    else if((workMode == SHOCK_DETECTION_MODE)&&(g_shockLevel != NO_SHOCK))
    {
      uint8_t data_len = 0;
      uint8_t collected_data[5] = { 0 };
      collected_data[data_len++] = 0x01;  // Shock/vibration detection
      if(g_shockLevel == SHOCK_LOW)
        collected_data[data_len++] = 0x00; // Shock level LOW
      else if(g_shockLevel == SHOCK_HIGH)
        collected_data[data_len++] = 0x01; // Shock level HIGH
      collected_data[data_len++] = (uint8_t)(g_shockCount >> 8);
      collected_data[data_len++] = (uint8_t)g_shockCount;

      Serial.printf("[SEND] date len:%d date:",data_len);
      for(int i = 0; i < data_len; i++)
      {
        Serial.printf("0x%02x,",collected_data[i]); 
      }
      Serial.println();

      if (api.lorawan.send(data_len, (uint8_t *) & collected_data, fport_event_uplink, api.lorawan.cfm.get(), 1))
          Serial.println("Sending is requested");
      else
          Serial.println("Sending failed");
      
      g_shockLevel = NO_SHOCK;
      g_shockCount = 0;
    }
}

void accIntHandle1(void)
{
  Serial.println("\r\naccIntHandle1");
  while (myIMU_lock) { delay(10); }
  myIMU_lock++;
  uint8_t dataRead;
  g_int1Flag = 1;

  myIMU.begin();
  //showIMUReg();
  myIMU.readRegister(&dataRead, LIS3DH_INT1_SRC);//cleared by reading

  if((workMode == MOTION_DETECTION_MODE) && (g_motionLevel != MOTION_HIGH))
  {
    g_motionLevel = MOTION_LOW;
    Serial.print("MOTION DETECT LOW\r\n");
  }
  else if((workMode == SHOCK_DETECTION_MODE) && (g_shockLevel != SHOCK_HIGH))
  {
    g_shockLevel = SHOCK_LOW;
    g_shockCount++;
    Serial.printf("SHOCK DETECT LOW g_shockCount = %d\r\n",g_shockCount);
  }
  else if(workMode == SHOCK_DETECTION_MODE)
  {
    g_shockCount++;
    Serial.printf("SHOCK DETECT LOW g_shockCount = %d\r\n",g_shockCount);
  }

  //myIMU.begin(LIS3D_INT1,LIS3D_INT2,accIntHandle1,accIntHandle2);
  myIMU.setMode(workMode);
  //showIMUReg();
  myIMU_lock--;
}

void accIntHandle2(void)
{
  Serial.println("\r\naccIntHandle2");
  while (myIMU_lock) { delay(10); }
  myIMU_lock++;
  uint8_t dataRead;
  g_int2Flag = 1;

  myIMU.begin();
  //showIMUReg();
  myIMU.readRegister(&dataRead, LIS3DH_INT2_SRC);//cleared by reading
  if(workMode == MOTION_DETECTION_MODE)
  {
    g_motionLevel = MOTION_HIGH;
    Serial.print("MOTION DETECT HIGH\r\n");
  }
  else if(workMode == SHOCK_DETECTION_MODE)
  {
    g_shockLevel = SHOCK_HIGH;
    g_shockCount++;
    Serial.printf("SHOCK DETECT HIGH g_shockCount = %d\r\n",g_shockCount);
  }

  //myIMU.begin(LIS3D_INT1,LIS3D_INT2,accIntHandle1,accIntHandle2);
  myIMU.setMode(workMode);
  //showIMUReg();
  myIMU_lock--;
}
