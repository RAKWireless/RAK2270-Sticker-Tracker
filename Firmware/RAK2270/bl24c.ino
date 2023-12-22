#include <Wire.h>

#define BL24C_ADDR  0x50

#if 0
#define BL24C_WP    PB13

void setup() 
{
  pinMode(BL24C_WP, OUTPUT);
  digitalWrite(BL24C_WP, LOW);
  
  Serial.begin(115200);
  Wire.begin();
  delay(100);
}

void loop() 
{
  uint8_t testResult = false;
  testResult = testBL24C();
  if(testResult == true)
  {
    Serial.printf("BL24C Test successful!\r\n"); 
  }
  else
  {
    Serial.printf("BL24C Test failed!\r\n"); 
  }
  delay(1000);
}
#endif

/*!
 *  @brief  Test BL24C.
 */
bool testBL24C(void)
{
  const char writeDate[25]="BL24C READ & WRITE TEST!";
  char readDate[25]="";
  
  //Serial.printf("[WRITE]:%s\r\n",writeDate);
  write(0x10,writeDate,strlen(writeDate));
  delay(500);
  read(0x10,readDate,strlen(writeDate));
  //Serial.printf("[READ]:%s\r\n",readDate);
  delay(500);
  if(memcmp(writeDate , readDate , sizeof(writeDate)) == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*!
 *  @brief  Write data of specified length to specified address through IIC.
 */
void write(uint32_t regAddr,const char *date , uint32_t size)
{
  byte error;
  uint8_t regH = (uint8_t)((regAddr >> 8) & 0xFF);
  uint8_t regL = (uint8_t)(regAddr & 0xFF);

  Wire.beginTransmission(BL24C_ADDR); // Setting up i2c connection.
  
  Wire.write(regH);
  Wire.write(regL);

  for (uint16_t i = 0; i < size; i++)
  {
    Wire.write(date[i]);
  }

  error = Wire.endTransmission(true); 
  delay(10);
}

/*!
 *  @brief  Read data of specified length to specified address through IIC.
 */
void read(uint32_t regAddr,char *date, uint32_t size)
{  
  byte error;
  uint8_t regH = (uint8_t)((regAddr >> 8) & 0xFF);
  uint8_t regL = (uint8_t)(regAddr & 0xFF);

  Wire.beginTransmission(BL24C_ADDR);
  Wire.write(regH); // Register address high
  Wire.write(regL); // Register address low
  error = Wire.endTransmission(); 

  Wire.requestFrom(BL24C_ADDR, size);
  
  uint16_t i = 0;
  while ( Wire.available() ) // Slave may send less than requested.
  {
    date[i] = Wire.read(); // Receive a byte as a proper uint8_t.
    i++;
  }
}
