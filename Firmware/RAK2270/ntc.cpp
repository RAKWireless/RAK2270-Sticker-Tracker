#include "./ntc.h"


#define NTC_EN    WB_A1
#define NTC_ADC   WB_A0

/**
 * @brief Get Temperature from NTC
 * Here is a ln relationship. T = 84.987-25.18*ln(10*V/(3.3-V))
 */
float getTemperature() 
{
  float max, ref;
  
  pinMode(NTC_EN, OUTPUT);
  digitalWrite(NTC_EN, HIGH);
  delay(200);

  analogReadResolution(10);
  pinMode(NTC_EN, OUTPUT);

    switch (udrv_adc_get_resolution()) {
        case UDRV_ADC_RESOLUTION_6BIT:
        {
            max = 64.0;
            break;
        }
        case UDRV_ADC_RESOLUTION_8BIT:
        {
            max = 256.0;
            break;
        }
        case UDRV_ADC_RESOLUTION_10BIT:
        default:
        {
            max = 1024.0;
            break;
        }
        case UDRV_ADC_RESOLUTION_12BIT:
        {
            max = 4096.0;
            break;
        }
        case UDRV_ADC_RESOLUTION_14BIT:
        {
            max = 16384.0;
            break;
        }
    }

    switch (udrv_adc_get_mode()) {
        case UDRV_ADC_MODE_DEFAULT:
        default:
        {
            #ifdef rak11720
            ref = 2.0;
            #else
            ref = 3.6;
            #endif
            break;
        }
        #ifdef rak11720
        case UDRV_ADC_MODE_1_5:
        {
            ref = 1.5;
            break;
        }
        #else
        case UDRV_ADC_MODE_3_3:
        {
            ref = 3.3;
            break;
        }
        case UDRV_ADC_MODE_3_0:
        {
            ref = 3.0;
            break;
        }
        case UDRV_ADC_MODE_2_4:
        {
            ref = 2.4;
            break;
        }
        case UDRV_ADC_MODE_1_8:
        {
            ref = 1.8;
            break;
        }
        case UDRV_ADC_MODE_1_2:
        {
            ref = 1.2;
            break;
        }
        #endif
    }

  float vbat = api.system.bat.get();
  uint16_t tv = analogRead(NTC_ADC);
  float tv2 = (tv * vbat)/max;
  float tmp = 84.987 - 25.18 *log(10*tv2/(vbat-tv2));
  
  delay(100);
  digitalWrite(NTC_EN, LOW);
  
  return tmp;
}

/**
 * @brief Calibrate Temperature
 * 1st: y = (0.00312f * x^2) + (0.927f * x) - 1.2f
 * 2nd: y = (0.976f * x) - 1.11
 */
float calibrateTemperature(float temp)
{
  float temp2 = (0.00312f * temp * temp) + (0.927f * temp) - 1.2f;
  temp2 = (0.976f * temp2) - 1.11;
  return temp2;
}
