#include <Arduino.h>
#include <SparkFunLIS3DH.h>
#include "./LoRaWAN.h"
#include "./lis3dh_md.h"
#include "./custom_at.h"
#include "./RAK2270.h"

#define USED_INT2             0 //INT2 reserve
#if USED_INT2
#define LIS3DH_INT2_CFG       0x34
#define LIS3DH_INT2_SRC       0x35
#define LIS3DH_INT2_THS       0x36
#define LIS3DH_INT2_DURATION  0x37
#endif

LIS3DH myIMU(I2C_MODE, 0x19);

void lis3dh_showConfigReg();
void lis3dh_setReg();
void lis3dh_get_md_data();
void lis3dh_INT1_handle();
void lis3dh_INT2_handle();


/* 
  BEGIN of linked list queue for MD event
 */
struct Node {
    uint32_t data;
    struct Node* next;
};

struct Node* head = NULL;
uint8_t send_event_flag = 0;

struct Node* createNode(uint32_t value);
void append(struct Node** head, uint32_t value);
void deleteNode(struct Node** head, struct Node* target);
struct Node* firstNode(struct Node* head);
struct Node* lastNode(struct Node* head);
int NodeSize(struct Node* head);
void display(struct Node* head);
void freeList(struct Node** head);
/* 
  END of linked list queue for MD event
 */

void lis3dh_showConfigReg()
{
  uint8_t readData = 0;

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

  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  Serial.printf("LIS3DH_INT1_CFG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_THS);
  Serial.printf("LIS3DH_INT1_THS: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT1_DURATION);
  Serial.printf("LIS3DH_INT1_DURATION: 0x%02X\r\n", readData);

#if USED_INT2
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);
  Serial.printf("LIS3DH_INT2_CFG: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_THS);
  Serial.printf("LIS3DH_INT2_THS: 0x%02X\r\n", readData);
  myIMU.readRegister(&readData, LIS3DH_INT2_DURATION);
  Serial.printf("LIS3DH_INT2_DURATION: 0x%02X\r\n", readData);
#endif
}

void lis3dh_setReg()
{
  uint8_t dataToWrite;
  uint16_t md_threshold = get_md_threshold();
  uint16_t md_sample_rate = get_md_sample_rate();
  Serial.printf("\r\nMD th: %d sr: %d\r\n", md_threshold, md_sample_rate);

  //LIS3DH_CTRL_REG1
  dataToWrite = 0;
  //dataToWrite |= 0x20; //ODR3-0, Data rate selection. Default value 00
                       //0000: Power down mode, 0001: 1Hz(N/L), 0010: 10 Hz(N/L), 0011: 25Hz(N/L), 0100: 50Hz(N/L)
                       //0101: 100 Hz(N/L), 0110: 200 Hz(N/L), 0111: 400 Hz(N/L), 1000: 1.6KHz(L), 1001: 1.25K Hz(N)/5K Hz(L)
  if (md_sample_rate == 0) {
    dataToWrite |= 0x10; //1Hz
  }
  else if (md_sample_rate == 1) {
    dataToWrite |= 0x20; //10Hz
  }
  else {
    dataToWrite |= 0x20; //10Hz
  }
  dataToWrite |= 0x08; //LPen, 0: normal mode, 1: low power mode
  dataToWrite |= 0x04; //Zen, 0: Z axis disable, 1: Z axis enable
  dataToWrite |= 0x02; //Yen, 0: Y axis disable, 1: Y axis enable
  dataToWrite |= 0x01; //Xen, 0: X axis disable, 1: X axis enable
  myIMU.writeRegister(LIS3DH_CTRL_REG1, dataToWrite);

  //LIS3DH_CTRL_REG2
  dataToWrite = 0;
  //dataToWrite |= 0x80; //HPM1-HPM0, High pass filter mode selection. 
  //dataToWrite |= 0x30; //HPCF2-HPCF1, High pass filter cut off frequency selection
  dataToWrite |= 0x08; //Filtered data selection. 0: internal filter bypassed; 1: data from internal filter sent to output register and FIFO.
  //dataToWrite |= 0x04; //High pass filter enabled for CLICK function 
  //dataToWrite |= 0x02; //High pass filter enabled for AOI function on interrupt 2
  dataToWrite |= 0x01; //High pass filter enabled for AOI function on interrupt 1
  myIMU.writeRegister(LIS3DH_CTRL_REG2, dataToWrite);

  //LIS3DH_CTRL_REG3
  //Choose source for pin 1
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click interrupt on INT1
  dataToWrite |= 0x40; //IA1 interrupt on INT1
  //dataToWrite |= 0x20; //IA2 interrupt on INT1
  //dataToWrite |= 0x10; //ZYXDA interrupt on INT1
  //dataToWrite |= 0x08; //321DA interrupt on INT1
  //dataToWrite |= 0x04; //FIFO watermark interrupt on INT1
  //dataToWrite |= 0x02; //FIFO overrun interrupt on INT1
  myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);

  //LIS3DH_CTRL_REG4
	dataToWrite = 0;
  //dataToWrite |= 0x80; //Block data update 
  //dataToWrite |= 0x40; //Big/little endian data selection 
  //dataToWrite |= 0x10; //FS1-FS0, Full scale selection. (00: +/- 2G; 01: +/- 4G; 10: +/- 8G; 11: +/- 16G)
  if (md_threshold >= 8000) {
    dataToWrite |= 0x30; //16g
  }
  else if (md_threshold >= 4000) {
    dataToWrite |= 0x20; //8g
  }
  else if (md_threshold >= 2000) {
    dataToWrite |= 0x10; //4g
  }
  else {
    dataToWrite |= 0x00; //2g
  }
  //dataToWrite |= 0x08; //High resolution output mode.
  //dataToWrite |= 0x04; //ST1-ST0, Self test enable.
  //dataToWrite |= 0x01; //SPI serial interface mode selection
  myIMU.writeRegister(LIS3DH_CTRL_REG4, dataToWrite);

  //LIS3DH_CTRL_REG5
  dataToWrite = 0;
  //Int1 latch interrupt and 4D on  int1 (preserve fifo en)
  //dataToWrite &= 0xF3; //Clear bits of interest
  dataToWrite |= 0x08; //Latch interrupt (Cleared by reading int1_src)
  //dataToWrite |= 0x04; //Pipe 4D detection from 6D recognition to int1?
  myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);

  //LIS3DH_CTRL_REG6
  //Choose source for pin 2 and both pin output inversion state
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click interrupt on pin2
  //dataToWrite |= 0x40; //IA1 interrupt on INT2
  //dataToWrite |= 0x20; //IA2 interrupt on INT2
  //dataToWrite |= 0x10; //boot on INT2 pin
  //dataToWrite |= 0x08; //activity interrupt on INT2 pin
  //dataToWrite |= 0x02; //INT1 and INT2 pin polarity
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);

  //LIS3DH_INT1_CFG
  dataToWrite = 0;
  //dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
  //dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
  //Set these to enable individual axes of generation source (or direction)
  // -- high and low are used generically
  dataToWrite |= 0x20;//Z high
  //dataToWrite |= 0x10;//Z low
  dataToWrite |= 0x08;//Y high
  //dataToWrite |= 0x04;//Y low
  dataToWrite |= 0x02;//X high
  //dataToWrite |= 0x01;//X low
  myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
  
  //LIS3DH_INT1_THS
  dataToWrite = 0;
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  uint8_t val = 0;
  if (md_threshold >= 8000) {
    val = md_threshold / 125;
  }
  else if (md_threshold >= 4000) {
    val = md_threshold / 63;
  }
  else if (md_threshold >= 2000) {
    val = md_threshold / 31;
  }
  else {
    val = md_threshold / 16;
  }
  if (val > 127) val = 127;
  dataToWrite |= val;
  myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
  
  //LIS3DH_INT1_DURATION
  dataToWrite = 0;
  //minimum duration of the interrupt, duration = value * (1000 ms / (sample rate))
  if (md_sample_rate == 0)
    dataToWrite |= 0x0;
  else if (md_sample_rate == 1)
    dataToWrite |= 0x0;
  myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);

#if USED_INT2 //INT2 reserve
  //LIS3DH_CTRL_REG6
  //Choose source for pin 2 and both pin output inversion state
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click interrupt on pin2
  //dataToWrite |= 0x40; //IA1 interrupt on INT2
  //dataToWrite |= 0x20; //IA2 interrupt on INT2
  //dataToWrite |= 0x10; //boot on INT2 pin
  //dataToWrite |= 0x08; //activity interrupt on INT2 pin
  //dataToWrite |= 0x02; //INT1 and INT2 pin polarity
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);

  //LIS3DH_INT2_CFG
  dataToWrite = 0;
  //dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
  //dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
  //Set these to enable individual axes of generation source (or direction)
  // -- high and low are used generically
  //dataToWrite |= 0x20;//Z high
  //dataToWrite |= 0x10;//Z low
  //dataToWrite |= 0x08;//Y high
  //dataToWrite |= 0x04;//Y low
  //dataToWrite |= 0x02;//X high
  //dataToWrite |= 0x01;//X low
  myIMU.writeRegister(LIS3DH_INT2_CFG, dataToWrite);
  
  //LIS3DH_INT2_THS
  dataToWrite = 0;
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  //dataToWrite |= 0x10; // 1/8 range
  //dataToWrite |= 0x1F;  // Threshold = 6000mg.
  myIMU.writeRegister(LIS3DH_INT2_THS, dataToWrite);
  
  //LIS3DH_INT2_DURATION
  dataToWrite = 0;
  //minimum duration of the interrupt
  //dataToWrite |= 0x0;
  myIMU.writeRegister(LIS3DH_INT2_DURATION, dataToWrite);
#endif

  //write CTRL_REG5 again after setting INT(AN3308 doc)
  //LIS3DH_CTRL_REG5
  dataToWrite = 0;
  //Int1 latch interrupt and 4D on int1 (preserve fifo en)
  //dataToWrite &= 0xF3; //Clear bits of interest
  dataToWrite |= 0x08; //Latch interrupt (Cleared by reading int1_src)
  //dataToWrite |= 0x04; //Pipe 4D detection from 6D recognition to int1?
  myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);

  return ;
}

void lis3dh_get_md_data()
{
  uint8_t dataRead;
  Serial.print("\r\nLIS3DH_INT1_SRC: 0x");
  myIMU.readRegister(&dataRead, LIS3DH_INT1_SRC);//cleared by reading
  Serial.println(dataRead, HEX);
  if(dataRead & 0x40) {
    Serial.print("Even detected:");
    if(dataRead & 0x02) Serial.print(" X+ ");
    if(dataRead & 0x01) Serial.print(" X- ");
    if(dataRead & 0x08) Serial.print(" Y+ ");
    if(dataRead & 0x04) Serial.print(" Y- ");
    if(dataRead & 0x20) Serial.print(" Z+ ");
    if(dataRead & 0x10) Serial.print(" Z- ");
    Serial.println();
  }

  //Get 3-axis data
  Serial.print("Accelerometer:");
  Serial.print(" X=");
  Serial.print(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y=");
  Serial.print(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z=");
  Serial.print(myIMU.readFloatAccelZ(), 4);
  Serial.println();
}

void lis3dh_INT1_handle()
{
  Serial.printf("\r\nlis3dh_INT1_handle\r\n");
  uint16_t md_duration = get_md_duration();
  uint8_t md_number = get_md_number();
  uint32_t md_silent_period = (uint32_t)get_md_silent_period();
  uint8_t dataRead;

  Wire.begin(); //contorl i2c after system wake up
  lis3dh_get_md_data();

  append(&head, millis());
  //Serial.printf("append\r\n");
  //Serial.printf("NodeSize %d\r\n", NodeSize(head));
  //Serial.printf("firstNode %d\r\n",firstNode(head)->data);
  //Serial.printf("lastNode %d\r\n",lastNode(head)->data);
  if (NodeSize(head) >= md_number)
  {
    //Serial.printf("%d <= %d ?\r\n",(lastNode(head)->data - firstNode(head)->data),md_duration);
    if ((lastNode(head)->data - firstNode(head)->data) <= md_duration) {
      send_event_flag = 1;
      freeList(&head);

      if (md_silent_period) {
        lis3dh_md_pause();

        if (api.system.timer.create(TIMER_MD_SILENT_PERIOD, (RAK_TIMER_HANDLER)lis3dh_md_resume, RAK_TIMER_ONESHOT) != true)
        {
          Serial.printf("Creating timer failed.\r\n");
          lis3dh_md_resume();
        } 
        else if (api.system.timer.start(TIMER_MD_SILENT_PERIOD, md_silent_period*1000, NULL) != true)
        {
          Serial.printf("Starting timer failed.\r\n");
          lis3dh_md_resume();
        }
      }
    }
    else
    {
      //Serial.printf("deleteNode\r\n");
      deleteNode(&head, firstNode(head));
    }
  }

  if (send_event_flag == 1)
  {
    //Serial.printf("loraSendMDEvent\r\n");
    loraSendMDEvent();
    send_event_flag = 0;
  }
}

#if USED_INT2 //INT2 reserve
void lis3dh_INT2_handle()
{
  Serial.printf("\r\nlis3dh_INT2_handle\r\n");
}
#endif

void lis3dh_md_pause()
{
  api.system.timer.stop(TIMER_MD_SILENT_PERIOD);
  freeList(&head);

  Wire.begin();
  uint8_t dataRead;
  myIMU.readRegister(&dataRead, LIS3DH_INT1_SRC);//cleared by reading
  myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x0);
}

void lis3dh_md_resume()
{
  Wire.begin();
  lis3dh_setReg();
  lis3dh_showConfigReg();
}

bool lis3dh_begin()
{
  //Accel sample rate and range effect interrupt time and threshold values!!!
  myIMU.settings.accelSampleRate = 0;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16

  myIMU.settings.adcEnabled = 0;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;

  if (myIMU.begin() != IMU_SUCCESS)
    return false;
  else
    return true;
}

void lis3dh_md_end()
{
  pinMode(LIS3D_INT1_PIN, INPUT);
  detachInterrupt(LIS3D_INT1_PIN);

  Wire.begin();
  lis3dh_md_pause();
  Wire.end();
}

void lis3dh_md_begin()
{
  if (lis3dh_begin())
  {
    lis3dh_md_pause();

    pinMode(LIS3D_INT1_PIN, INPUT_PULLDOWN);
    attachInterrupt(LIS3D_INT1_PIN, lis3dh_INT1_handle, RISING);

    lis3dh_md_resume();
  }
  else
  {
    Serial.println("LIS3DH initialize fail...");
  }
}

void lis3dh_init()
{
  if (lis3dh_begin())
  {
    lis3dh_md_end();
  }
  else
  {
    Serial.println("LIS3DH initialize fail...");
  }
}

bool lis3dh_factory_test()
{
  bool ret = lis3dh_begin();
  if (ret)
  {
    lis3dh_md_pause();
    Wire.end();
  }
  return ret;
}

/* 
  BEGIN of linked list queue for MD event
 */
// Function to create a new node with a given value
struct Node* createNode(uint32_t value) {
    struct Node* newNode = (struct Node*)malloc(sizeof(struct Node));
    if (!newNode) {
        printf("Memory allocation error\n");
        return NULL;
    }
    newNode->data = value;
    newNode->next = NULL;
    return newNode;
}

// Function to append a node to the end of the linked list
void append(struct Node** head, uint32_t value) {
    struct Node* newNode = createNode(value);
    if (*head == NULL) {  // If list is empty, make new node the head
        *head = newNode;
    } else {
        struct Node* temp = *head;
        while (temp->next != NULL) {
            temp = temp->next;
        }
        temp->next = newNode;
    }
}

// Function to delete a node with a specific value
void deleteNode(struct Node** head, struct Node* target) {
    struct Node* temp = *head;
    struct Node* prev = NULL;

    // Check if head node holds the value to be deleted
    if (temp != NULL && temp == target) {
        *head = temp->next;  // Change head
        free(temp);          // Free old head
        return;
    }

    // Search for the node to delete, keeping track of the previous node
    while (temp->next != NULL && temp->next != target) {
        prev = temp;
        temp = temp->next;
    }

    // If value not found in the list
    if (temp == NULL) {
        //Serial.printf("target not found in the list.\r\n");
        return;
    }

    // Unlink the node from the linked list and free its memory
    prev->next = temp->next;
    free(temp);
}

// Function to search for the first node
struct Node* firstNode(struct Node* head) {
    return head;
}

// Function to search for the last node
struct Node* lastNode(struct Node* head) {
    struct Node* temp = head;
    struct Node* prev = NULL;
    while (temp != NULL) {
        prev = temp;
        temp = temp->next;
    }
    return prev;
}

// Function to get the size of the linked list
int NodeSize(struct Node* head) {
    int count = 0;
    struct Node* temp = head;
    while (temp != NULL) {
        count++;
        temp = temp->next;
    }
    return count;
}

// Function to display the linked list
void display(struct Node* head) {
    struct Node* temp = head;
    while (temp != NULL) {
        printf("%d -> ", temp->data);
        temp = temp->next;
    }
    printf("NULL\n");
}

// Function to free all nodes in the list
void freeList(struct Node** head) {
    struct Node* temp = *head;
    struct Node* prev = NULL;
    while (temp != NULL) {
        prev = temp;
        temp = temp->next;
        free(prev);
    }
    *head = temp;
}
/* 
  END of linked list queue for MD event
 */
