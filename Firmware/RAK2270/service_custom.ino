#include "service_custom.h"

void service_custom_read_user_metadata(stored_metadata * metadata)
{
  udrv_flash_read(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)metadata);
}

void service_custom_write_user_metadata(stored_metadata * metadata)
{
  udrv_flash_write(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)metadata);
}

int32_t service_custom_write_user_data (void *buff, uint32_t len,bool cached) 
{
    stored_metadata metadata;
    udrv_flash_read(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)&metadata);
    if(metadata.write_pos == 0xFFFF && metadata.read_pos == 0xFFFF)
    {
        metadata.write_pos = 0;
        metadata.read_pos = 0;
    }
    if(cached)
    {
        if(len < USER_CACHE_BUF_SIZE)
        {
            if(len + cached_buf_first < USER_CACHE_BUF_SIZE)
            {
                memcpy(cached_buf+cached_buf_first,buff,sizeof(stored_data)*len);
                cached_buf_first += len;
                return UDRV_RETURN_OK;
            }
            else
            {
                memcpy(cached_buf+cached_buf_first,buff,sizeof(stored_data)*(USER_CACHE_BUF_SIZE - len));
                cached_buf_first = USER_CACHE_BUF_SIZE - len + cached_buf_first;
                metadata.read_pos += USER_CACHE_BUF_SIZE;
                return udrv_flash_write(USER_DATA_NVM_ADDR, sizeof(cached_buf), (uint8_t *)&cached_buf);
            }
        }
    }
    else
    {
        if(len + metadata.write_pos < USER_BUFFER_SIZE)
        {
            udrv_flash_write(USER_DATA_NVM_ADDR + (sizeof(stored_data)*(metadata.write_pos+len-1)), sizeof(stored_data)*len, (uint8_t *)buff);
            metadata.write_pos +=len;
            if(metadata.write_pos >= USER_BUFFER_SIZE )
                metadata.write_pos = 0;
        }
        else
        {
            udrv_flash_write(USER_DATA_NVM_ADDR + (sizeof(stored_data)*(metadata.write_pos)), sizeof(stored_data)*(USER_BUFFER_SIZE-metadata.write_pos), (uint8_t *)buff);
            metadata.write_pos = len + metadata.write_pos - USER_BUFFER_SIZE;
            udrv_flash_write(USER_DATA_NVM_ADDR , sizeof(stored_data)*(metadata.write_pos), (uint8_t *)(buff+USER_BUFFER_SIZE-metadata.write_pos));
        }
    }
    //metadata.read_pos++;

    return udrv_flash_write(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)&metadata);
}

int32_t service_custom_read_user_data (void *buff,uint16_t offset,uint32_t len) 
{
    //stored_metadata metadata;
    //udrv_flash_read(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)&metadata);
    if(len + offset < USER_BUFFER_SIZE)
    {
        return udrv_flash_read(USER_DATA_NVM_ADDR + (sizeof(stored_data)*(offset+len-1)), sizeof(stored_data)*len, (uint8_t *)buff);
        //metadata.read_pos +=len;
    }
    else
    {
        return -UDRV_BUFF_OVERFLOW;
        //udrv_flash_read(USER_DATA_NVM_ADDR + (sizeof(stored_data)*(metadata.read_pos)), sizeof(stored_data)*(USER_BUFFER_SIZE-len), (uint8_t *)buff);
        //metadata.read_pos = len + metadata.read_pos - USER_BUFFER_SIZE;
        //udrv_flash_read(USER_DATA_NVM_ADDR , sizeof(stored_data)*(metadata.read_pos), (uint8_t *)buff);
    }
    //return udrv_flash_write(SERVICE_NVM_USER_DATA_NVM_ADDR, sizeof(stored_metadata), (uint8_t *)&metadata);

}
