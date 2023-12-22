#include "LoRaWAN.h"

/** Packet buffer for sending */
uint8_t collected_data[64] = { 0 };

void get_data_seq(uint8_t command)
{
    stored_data buf;
    uint16_t buf_ptr=0;
    uint16_t seq_no = 0;
    uint16_t exceed = USER_BUFFER_SIZE+1;
    for(int i = 0;i<USER_BUFFER_SIZE;i++)
    {
        if(flash_buf[i] != exceed)
            flash_buf[i] = exceed;
        else
            break;
    }
    flash_buf_pos = 0;
    for(int i=0;i<USER_BUFFER_SIZE;i++)
    {
        udrv_flash_read(USER_DATA_NVM_ADDR + i*sizeof(buf), sizeof(buf), (uint8_t *)&buf);
        //seq_no = buf.seq_no & ((1<<13)-1);
        seq_no = buf.seq_no & 0xFFF;
        if(command == 0)
        {
            if((seq_no == i+1) && (buf.timestamp > start_time) && (buf.timestamp < end_time))
            {
                flash_buf[buf_ptr++] = i;
            }
        }
        else if (command == 1)
        {
            //Serial.printf("i=%d, seq_no=%d, buf.seq_no=%d, start_seq_no=%d, end_seq_no=%d\r\n", (int)i, (int)seq_no, (int)buf.seq_no, (int)start_seq_no, (int)end_seq_no);
            if((seq_no == i+1) && (buf.seq_no >= start_seq_no) && (buf.seq_no <= end_seq_no))
            {
                flash_buf[buf_ptr++] = i;
            }
        }
        else if (command == 4)
        {
            if((seq_no == i+1) && (buf.timestamp > start_time_bin) && (buf.timestamp < end_time_bin))
            {
                flash_buf[buf_ptr++] = i;
            }
        }
        else if (command == 5)
        {
            if((seq_no == i+1) && (buf.seq_no >= start_seq_no_bin) && (buf.seq_no <= end_seq_no_bin))
            {
                flash_buf[buf_ptr++] = i;
            }
        }
    }
    if(command == 4 || command == 5)
    {
        binary_space_partition(flash_buf,buf_ptr,exceed);
    }
}
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
            case 0: // 通过时间 TimeStamp 进行数据检索
            {
              if (custom_id == CUSTOM_NOVALABS) {
                if(data->BufferSize != 9)
                    Serial.printf("command 0x00 error\r\n");
                else if (get_event_mode() & EVENT_RETRIEVAL_UPLINK)
                {
                    start_time = (data->Buffer[4] & 0xFF) + ((data->Buffer[3] & 0xFF) << 8) + ((data->Buffer[2] & 0xFF) << 16) + ((data->Buffer[1] & 0xFF) << 24);
                    end_time = (data->Buffer[8] & 0xFF) + ((data->Buffer[7] & 0xFF) << 8) + ((data->Buffer[6] & 0xFF) << 16) + ((data->Buffer[5] & 0xFF) << 24);
                    uint64_t current_time = millis()/1000;
                    api.system.timer.stop(RAK_TIMER_1);
                    get_data_seq(command);
                    send_data();
                    if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)send_data, RAK_TIMER_PERIODIC) != true) 
                        Serial.printf("Creating timer failed.\r\n");
                    else if (api.system.timer.start(RAK_TIMER_1, RETRIEVAL_UPLINK_INTERVAL, NULL) != true) 
                        Serial.printf("Starting timer failed.\r\n"); 
                }
              }
              break;
            }
            case 1: // 通过序列号 进行数据检索
            {
              if (custom_id == CUSTOM_NOVALABS) {
                if(data->BufferSize != 5)
                    Serial.printf("command 0x01 error\r\n");
                else if (get_event_mode() & EVENT_RETRIEVAL_UPLINK)
                {
                    start_seq_no = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
                    end_seq_no = (data->Buffer[4] & 0xFF) + ((data->Buffer[3] & 0xFF) << 8);
                    api.system.timer.stop(RAK_TIMER_1);
                    get_data_seq(command);
                    send_data();
                    if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)send_data, RAK_TIMER_PERIODIC) != true)
                       Serial.printf("Creating timer failed.\r\n");
                    else if (api.system.timer.start(RAK_TIMER_1, RETRIEVAL_UPLINK_INTERVAL, NULL) != true)
                       Serial.printf("Starting timer failed.\r\n");

                }
              }
              break;
            }
            case 2: // 修改周期上发数据时间间隔
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
            case 3: // 修改 ACC 触发 工作模式
            {
              if (custom_id == CUSTOM_NOVALABS) {
                if(data->BufferSize != 2)
                    Serial.printf("command 0x03 error\r\n");
                else
                {
                    tmp_8 = (data->Buffer[1] & 0xFF);
                    //if(tmp_8 != 0 && tmp_8 != 1)
                    //    Serial.printf("command 0x03 parameter error\r\n");
                    //else
                    //{
                        uint8_t event_mode = get_event_mode();
                        if (data->Buffer[1] & EVENT_MOTION_DETECTION) {
                          workMode = MOTION_DETECTION_MODE;
                          Serial.printf("Motion detection enable\r\n");
                          Serial.printf("Shock detection disable\r\n");
                          event_mode |= MOTION_DETECTION_MODE;
                          event_mode &= ~EVENT_SHOCK_DETECTION;
                          attachInterrupt(LIS3D_INT1, accIntHandle2, RISING);
                          attachInterrupt(LIS3D_INT2, accIntHandle1, RISING);
                        }
                        else if (data->Buffer[1] & EVENT_SHOCK_DETECTION) {
                          workMode = SHOCK_DETECTION_MODE;
                          Serial.printf("Motion detection disable\r\n");
                          Serial.printf("Shock detection enable\r\n");
                          event_mode &= ~MOTION_DETECTION_MODE;
                          event_mode |= EVENT_SHOCK_DETECTION;
                          attachInterrupt(LIS3D_INT1, accIntHandle2, RISING);
                          attachInterrupt(LIS3D_INT2, accIntHandle1, RISING);
                        }
                        else {
                          //workMode = NONE_MODE;
                          workMode = MOTION_DETECTION_MODE;
                          Serial.printf("Motion detection disable\r\n");
                          Serial.printf("Shock detection disable\r\n");
                          event_mode &= ~MOTION_DETECTION_MODE;
                          event_mode &= ~EVENT_SHOCK_DETECTION;
                          detachInterrupt(LIS3D_INT1);
                          detachInterrupt(LIS3D_INT2);
                        }

                        if (data->Buffer[1] & EVENT_RETRIEVAL_UPLINK) {
                          Serial.printf("Retrieval uplink enable\r\n");
                          event_mode |= EVENT_RETRIEVAL_UPLINK;
                        }
                        else {
                          Serial.printf("Retrieval uplink disable\r\n");
                          event_mode &= ~EVENT_RETRIEVAL_UPLINK;
                        }
                        set_event_mode(event_mode);
                        g_motionLevel = NO_MOVE;
                        g_shockLevel = NO_SHOCK;
                        g_shockCount = 0;

                        while (myIMU_lock) { delay(10); }
                        myIMU_lock++;
                        myIMU.begin();
                        myIMU.setMode(workMode);
                        myIMU_lock--;
                    //}
                    if (api.lorawan.send(data->BufferSize, (uint8_t *) data->Buffer, fport, api.lorawan.cfm.get(), retry))
                        Serial.println("Sending is requested");
                    else
                        Serial.println("Sending failed");
                }
              }
              break;
            }
            case 4: // 通过时间 数据检索，使用 Bsp 序列进行上传
            {
              if (custom_id == CUSTOM_NOVALABS) {
                if(data->BufferSize != 9)
                    Serial.printf("command 0x04 error\r\n");
                else if (get_event_mode() & EVENT_RETRIEVAL_UPLINK)
                {
                    start_time_bin = (data->Buffer[4] & 0xFF) + ((data->Buffer[3] & 0xFF) << 8) + ((data->Buffer[2] & 0xFF) << 16) + ((data->Buffer[1] & 0xFF) << 24);
                    end_time_bin = (data->Buffer[8] & 0xFF) + ((data->Buffer[7] & 0xFF) << 8) + ((data->Buffer[6] & 0xFF) << 16) + ((data->Buffer[5] & 0xFF) << 24);
                    api.system.timer.stop(RAK_TIMER_1);
                    get_data_seq(command);
                    send_data();
                    if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)send_data, RAK_TIMER_PERIODIC) != true)
                       Serial.printf("Creating timer failed.\r\n");
                    else if (api.system.timer.start(RAK_TIMER_1, RETRIEVAL_UPLINK_INTERVAL, NULL) != true)
                       Serial.printf("Starting timer failed.\r\n");
                }
              }
              break;
            }
            case 5: // 通过序列号 数据检索，使用 Bsp 序列进行上传
            {
              if (custom_id == CUSTOM_NOVALABS) {
                if(data->BufferSize != 5)
                    Serial.printf("command 0x05 error\r\n");
                else if (get_event_mode() & EVENT_RETRIEVAL_UPLINK)
                {
                    start_seq_no_bin = (data->Buffer[2] & 0xFF) + ((data->Buffer[1] & 0xFF) << 8);
                    end_seq_no_bin = (data->Buffer[4] & 0xFF) + ((data->Buffer[3] & 0xFF) << 8);
                    api.system.timer.stop(RAK_TIMER_1);
                    get_data_seq(command);
                    send_data();
                    if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)send_data, RAK_TIMER_PERIODIC) != true)
                       Serial.printf("Creating timer failed.\r\n");
                    else if (api.system.timer.start(RAK_TIMER_1, RETRIEVAL_UPLINK_INTERVAL, NULL) != true)
                       Serial.printf("Starting timer failed.\r\n");
                }
              }
              break;
            }
            case 6: // 修改 ACC 触发 工作模式
            {
              if (custom_id == CUSTOM_NOVALABS) {
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
                  if (api.lorawan.send(data->BufferSize, (uint8_t *) data->Buffer, fport, api.lorawan.cfm.get(), retry))
                    Serial.println("Sending is requested");
                  else
                    Serial.println("Sending failed");
                }
              }
              break;
            }
            default:
                Serial.printf("Invalid command!\r\n");
        }
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

void joinCallback(int32_t status)
{
  Serial.printf("Join status: %d\r\n", status);
  if (status == 0) {
    set_lora_network_status(JOINED);
    reset_last_send_ok_time();

    if (custom_id == CUSTOM_STANDARD) {
      api.system.timer.stop(RAK_TIMER_3);
      if (api.system.timer.create(RAK_TIMER_3, (RAK_TIMER_HANDLER)linkcheck_handler, RAK_TIMER_PERIODIC) != true)
        Serial.printf("Creating timer failed.\r\n");
      else if (api.system.timer.start(RAK_TIMER_3, DEFAULT_LINKCHECK_INTERVAL, NULL) != true)
        Serial.printf("Starting timer failed.\r\n");
    }

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

void sendCallback(int32_t status)
{

}

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

void loraWanInit()
{
//  Serial.begin(115200, RAK_AT_MODE);
  
//  if(api.lorawan.nwm.get() != 1)
//  {
//    Serial.printf("Set Node device work mode %s\r\n",
//    api.lorawan.nwm.set(1) ? "Success" : "Fail");
//    api.system.reboot();
//  }
//  
//  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) 
//  {
//    Serial.printf("LoRaWan OTAA - set device class is incorrect! \r\n");
//    return;
//  }
//  if (!api.lorawan.njm.set(RAK_LORA_OTAA))  // Set the network join mode to OTAA
//  {
//    Serial.printf("LoRaWan OTAA - set network join mode is incorrect! \r\n");
//    return;
//  }

//  if (!api.lorawan.adr.set(false)) 
//  {
//    Serial.printf("LoRaWan OTAA - set adaptive data rate is incorrect! \r\n");
//    return;
//  }
//  if (!api.lorawan.rety.set(1)) 
//  {
//    Serial.printf("LoRaWan OTAA - set retry times is incorrect! \r\n");
//    return;
//  }
//  if (!api.lorawan.cfm.set(0))
//  {
//    Serial.printf("LoRaWan OTAA - set confirm mode is incorrect! \r\n");
//    return;
//  }
//
//  /** Check LoRaWan Status*/
//  if(api.lorawan.dcs.get() == 1)
//  {
//    api.lorawan.dcs.set(0);
//  }


  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
  service_lora_register_linkcheck_cb(linkcheckCallback);

  
//  if (!api.lorawan.join())  // Join to Gateway
//  {
//    Serial.printf("LoRaWan OTAA - join fail! \r\n");
//    return;
//  }
}
/*
 * @note: 用于处理接收下发指令后上发数据。
 */
void send_data(void)
{
  Serial.printf("send_data\r\n");
  uint8_t collected_data[20] = { 0 };
  uint8_t data_len = 0;
  if(flash_buf[flash_buf_pos] == USER_BUFFER_SIZE+1)
  {
    Serial.println("Sending buf is empty");
    api.system.timer.stop(RAK_TIMER_1);
    return;
  }
  stored_data read_data;
  service_custom_read_user_data(&read_data,flash_buf[flash_buf_pos++],1);
  
  collected_data[data_len++] = (uint8_t)(read_data.seq_no >> 8);
  collected_data[data_len++] = (uint8_t)read_data.seq_no;
  collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0xFF000000) >> 24);;
  collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0x00FF0000) >> 16);
  collected_data[data_len++] = (uint8_t)((read_data.timestamp & 0x0000FF00) >> 8);
  collected_data[data_len++] = (uint8_t)(read_data.timestamp & 0x000000FF);
  collected_data[data_len++] = (uint8_t)(read_data.payload >> 8);
  collected_data[data_len++] = (uint8_t)read_data.payload;
  
  if (api.lorawan.send(data_len, (uint8_t *) & collected_data, fport_retrieval_uplink, api.lorawan.cfm.get(), 1))
    Serial.println("Sending is requested");
  else
    Serial.println("Sending failed");
}

/*
 * @note: 用于周期上发数据。
 */
void loraSendDate()
{
  uint8_t *bufPtr;
  uint8_t data_len = 0;
  
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
    bufPtr = g_solution_data.getBuffer();
    data_len = g_solution_data.getSize();
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
 * @note: 用产测上发数据。
 */
void loraSendTempDate(uint8_t len, uint8_t *buf)
{
  
  if (api.lorawan.njs.get() == 0) 
  {
    Serial.println("Wait for LoRaWAN join...");
    if (!api.lorawan.join(1,0,7,3)) // 加入网络 手动加入 每8s尝试一次 尝试5次
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
