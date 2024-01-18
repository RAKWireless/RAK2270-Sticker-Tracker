#include "./custom_at.h"


uint32_t  g_txInterval;

/**
 * @brief Get setting from flash
 */
bool get_at_setting(uint32_t setting_type)
{
  uint8_t flash_value[18];
  switch (setting_type)
  {
  case SEND_FREQ_OFFSET:
    if (!api.system.flash.get(SEND_FREQ_OFFSET, flash_value, 5))
    {
      g_txInterval = DEFAULT_TXINTERVAL;
      save_at_setting(SEND_FREQ_OFFSET);
      return false;
    }
    if (flash_value[4] != 0xAA)
    {
      g_txInterval = DEFAULT_TXINTERVAL;
      save_at_setting(SEND_FREQ_OFFSET);
      return false;
    }

    g_txInterval = 0;
    g_txInterval |= flash_value[0] << 0;
    g_txInterval |= flash_value[1] << 8;
    g_txInterval |= flash_value[2] << 16;
    g_txInterval |= flash_value[3] << 24;

    if((g_txInterval > 1440) || (g_txInterval == 0))
    {
      g_txInterval = DEFAULT_TXINTERVAL;
      save_at_setting(SEND_FREQ_OFFSET);
      return false;
    }
    
    return true;
    break;

  default:
    return false;
  }
}

/**
 * @brief Save setting to flash
 */
bool save_at_setting(uint32_t setting_type)
{
  uint8_t flash_value[18] = {0};
  bool wr_result = false;
  switch (setting_type)
  {
    case  SEND_FREQ_OFFSET:
          flash_value[0] = (uint8_t)(g_txInterval >> 0);
          flash_value[1] = (uint8_t)(g_txInterval >> 8);
          flash_value[2] = (uint8_t)(g_txInterval >> 16);
          flash_value[3] = (uint8_t)(g_txInterval >> 24);
          flash_value[4] = 0xAA; // dirty byte for checking

          wr_result = api.system.flash.set(SEND_FREQ_OFFSET, flash_value, 5);
          wr_result = true;
          return wr_result;
          break;
    default:
          return false;
          break;
  }
  return false;
}

/**
 * @brief Handler for send frequency AT commands
 *
 */
int freq_send_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
//  char ble_data[50];
  if (param->argc == 1 && !strcmp(param->argv[0], "?"))
  {
    Serial.print(cmd);
    Serial.printf("=%ld minutes\r\n", g_txInterval);
  }
  else if (param->argc == 1)
  {
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      if (!isdigit(*(param->argv[0] + i)))
      {
        Serial.printf("%d is no digit", i);
        return AT_PARAM_ERROR;
      }
    }

    uint32_t new_send_freq = strtoul(param->argv[0], NULL, 10);

    if((new_send_freq > 1440) || (new_send_freq == 0))
    {
      Serial.printf("Does not exceed 1-1440");
      return AT_PARAM_ERROR;
    }
    
    g_txInterval = new_send_freq;

    save_at_setting(SEND_FREQ_OFFSET);
  }
  else
  {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Add send-frequency AT command
 */
bool init_frequency_at(void)
{
  return api.system.atMode.add((char *)"SENDFREQ",
      (char *)"Set/Get the frequent automatic sending time values in seconds, 1-1440 minutes",
      (char *)"SENDFREQ", freq_send_handler);
}



uint8_t g_FACTORY_MODE = DEFAULT_FACTORY_MODE;

/**
 * @brief get factory mode
 */
uint8_t get_factory_mode(void)
{
  return g_FACTORY_MODE;
}

/**
 * @brief set factory mode
 */
bool set_factory_mode(uint8_t mode)
{
  if (api.system.flash.set(FACTORY_MODE_OFFSET, &mode, 1) == false) {
    return false;
  }
  g_FACTORY_MODE = mode;
  return true;
}

/**
 * @brief handle factory mode
 */
int factory_mode_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_factory_mode()?"ON":"OFF");
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint8_t tmp = strtoul(param->argv[0], NULL, 10);
    if (tmp != FACTORY_MODE_OFF && tmp != FACTORY_MODE_ON) {
      return AT_PARAM_ERROR;
    }

    if (set_factory_mode(tmp) == false) {
      return AT_ERROR;
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for factory mode
 */
void register_factory_mode_atcmd(void)
{
  api.system.atMode.add("FMODE", "Set/Get FACTORY_MODE (0 OFF, 1 ON)", "FMODE", factory_mode_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize factory mode
 */
void init_factory_mode()
{
  if (api.system.flash.get(FACTORY_MODE_OFFSET, &g_FACTORY_MODE, 1) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  register_factory_mode_atcmd();
}
