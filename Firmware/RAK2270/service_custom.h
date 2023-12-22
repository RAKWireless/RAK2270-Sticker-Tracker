#ifndef __SERVICE_CUSTOM_H__
#define __SERVICE_CUSTOM_H__

#include "Arduino.h"
#include "udrv_flash.h"


/*
 * RAK2270 Flash Layout for User Data
 *   0x8000000  -------------------
 *              | system_reserved |
 *   0x8036000  -------------------                        ---
 *              | stored_metadata |  16 bytes               |
 *              -------------------                         |
 *              | stored_data     |  30720 bytes = 3840*8   | 36K bytes
 *              |                 |  36336 bytes = 3028*12  |
 *   0x803EE00  -------------------                         |
 *              | custom_config   |  512 bytes              |
 *   0x803F000  -------------------                        ---
 *              | system_config   |  2K bytes
 *   0x803F800  -------------------
 *              | default_config  |  2K bytes
 *   0x8040000  -------------------
 */


//#define USER_FLASH_SIZE           32768
//#define USER_DATA_SIZE            8
#define USER_CACHE_BUF_SIZE       1024
#define USER_BUFFER_SIZE          3840 //for stored_data(8 bytes)
//#define USER_BUFFER_SIZE          3028 //for stored_data(12 bytes)
#define USER_DATA_NVM_ADDR        (SERVICE_NVM_USER_DATA_NVM_ADDR + sizeof(stored_metadata))
#define RETRIEVAL_UPLINK_INTERVAL 7000

typedef struct 
{
  uint16_t write_pos;
  uint16_t read_pos;
  uint32_t reserved;
  uint32_t reserved1;
  uint32_t reserved2;
}stored_metadata; // 16bytes

typedef struct
{
  uint16_t seq_no;
  uint16_t payload;
  uint32_t timestamp;
//  uint16_t vbat;
//  uint8_t eventStatus;
//  uint8_t reserved;
}stored_data;


void service_custom_read_user_metadata(stored_metadata * metadata);
int32_t service_custom_write_user_data (void *buff, uint32_t len,bool cached);

#endif
