#include "./custom_at.h"
#include "./lis3dh_md.h"
#include "./LoRaWAN.h"
#include "./RAK2270.h"


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

    if (get_system_stage() == PERIODIC_UPLINK_STAGE)
    {
      api.system.timer.stop(TIMER_PERIODIC_UPLINK);
      if (api.system.timer.create(TIMER_PERIODIC_UPLINK, (RAK_TIMER_HANDLER)periodic_uplink_handler, RAK_TIMER_PERIODIC) != true) {
        Serial.printf("Failed to creat timer.\r\n");
      }
      else if (api.system.timer.start(TIMER_PERIODIC_UPLINK, g_txInterval * 60000, NULL) != true) {
        Serial.printf("Failed to start timer.\r\n");
      }
    }
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
      (char *)"Set/Get the periodic uplink interval (1-1440 minutes)",
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
  api.system.atMode.add("FMODE", "Set/Get the Factory Mode status (0: OFF, 1: ON)", "FMODE", factory_mode_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize factory mode
 */
void init_factory_mode()
{
  if (api.system.flash.get(FACTORY_MODE_OFFSET, &g_FACTORY_MODE, 1) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  else
  {
    if (g_FACTORY_MODE == 0xFF)
    {
      set_factory_mode(DEFAULT_FACTORY_MODE);
    }
  }
  register_factory_mode_atcmd();
}

uint8_t g_NTC_CALIBRATION_MODE = DEFAULT_NTC_CALIBRATION_MODE;

/**
 * @brief get ntc calibration mode
 */
uint8_t get_ntc_calibration_mode(void)
{
  return g_NTC_CALIBRATION_MODE;
}

/**
 * @brief set ntc calibration mode
 */
bool set_ntc_calibration_mode(uint8_t mode)
{
  if (api.system.flash.set(NTC_CALIBRATION_MODE_OFFSET, &mode, 1) == false) {
    return false;
  }
  g_NTC_CALIBRATION_MODE = mode;
  return true;
}

/**
 * @brief handle ntc calibration mode
 */
int ntc_calibration_mode_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.printf("%d\r\n",get_ntc_calibration_mode());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint8_t tmp = strtoul(param->argv[0], NULL, 10);
    if (tmp != 0 && tmp != 1) {
      return AT_PARAM_ERROR;
    }

    if (set_ntc_calibration_mode(tmp) == false) {
      return AT_ERROR;
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for ntc calibration mode
 */
void register_ntc_calibration_mode_atcmd(void)
{
  api.system.atMode.add("NCMODE", "Set/Get the NTC Calibration Mode status (0: OFF, 1: ON)", "NCMODE", ntc_calibration_mode_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize ntc calibration mode
 */
void init_ntc_calibration_mode()
{
  if (api.system.flash.get(NTC_CALIBRATION_MODE_OFFSET, &g_NTC_CALIBRATION_MODE, 1) == false)
  {
    Serial.print("Reading flash data failure...\r\n");
  }
  else
  {
    if (g_NTC_CALIBRATION_MODE == 0xFF)
    {
      set_ntc_calibration_mode(DEFAULT_NTC_CALIBRATION_MODE);
    }
  }
  register_ntc_calibration_mode_atcmd();
}

uint8_t g_DEVELOPMENT_MODE = DEFAULT_DEVELOPMENT_MODE;

/**
 * @brief get development mode
 */
uint8_t get_development_mode(void)
{
  return g_DEVELOPMENT_MODE;
}

/**
 * @brief set development mode
 */
bool set_development_mode(uint8_t mode)
{
  g_DEVELOPMENT_MODE = mode;
  return true;
}

/**
 * @brief handle development mode
 */
int development_mode_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.printf("%d\r\n",get_development_mode());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint8_t tmp = strtoul(param->argv[0], NULL, 10);
    if (tmp != 0 && tmp != 1) {
      return AT_PARAM_ERROR;
    }

    if (set_development_mode(tmp) == false) {
      return AT_ERROR;
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for development mode
 */
void register_development_mode_atcmd(void)
{
  api.system.atMode.add("DMODE", "Set/Get the Development Mode (0: OFF, 1: ON)", "DMODE", development_mode_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize development mode
 */
void init_development_mode()
{
  set_development_mode(DEFAULT_DEVELOPMENT_MODE);
  register_development_mode_atcmd();
}


/**
 * @brief handle re-join
 */
int rejoin_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 0) {
    if (get_factory_mode() == FACTORY_MODE_OFF)
      return_to_join_stage();
    return AT_OK;
  }
  else {
    return AT_PARAM_ERROR;
  }
}

/**
 * @brief Register ATC Command for re-join
 */
void register_rejoin_atcmd(void)
{
  api.system.atMode.add("REJOIN", "Return To Join Stage", "JOIN", rejoin_handle, RAK_ATCMD_PERM_READ);
}

uint32_t g_JOIN_INTERVAL = DEFAULT_JOIN_INTERVAL;

/**
 * @brief get join interval
 */
uint32_t get_join_interval(void)
{
  return g_JOIN_INTERVAL;
}

/**
 * @brief set join interval
 */
bool set_join_interval(uint32_t interval)
{
  if (api.system.flash.set(JOIN_INTERVAL_OFFSET, (uint8_t*)&interval, 4) == false) {
    return false;
  }
  g_JOIN_INTERVAL = interval;
  return true;
}

/**
 * @brief handle join interval
 */
int join_interval_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_join_interval());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint32_t tmp = strtoul(param->argv[0], NULL, 10);
    if (tmp < 1 || tmp > 1440) {
      return AT_PARAM_ERROR;
    }

    if (set_join_interval(tmp) == false) {
      return AT_ERROR;
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for join interval
 */
void register_join_interval_atcmd(void)
{
  api.system.atMode.add("JINTV", "Set/Get the periodic join interval (1-1440 minutes)", "JOIN", join_interval_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize join interval
 */
void init_join_interval()
{
  if (api.system.flash.get(JOIN_INTERVAL_OFFSET, (uint8_t*)&g_JOIN_INTERVAL, 4) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  else
  {
    if (g_JOIN_INTERVAL == 0xFFFFFFFF)
    {
      set_join_interval(DEFAULT_JOIN_INTERVAL);
    }
  }
  register_join_interval_atcmd();
}

uint32_t g_LINKCHECK_TIMEOUT = DEFAULT_LINKCHECK_TIMEOUT;

/**
 * @brief get linkcheck_timeout
 */
uint32_t get_linkcheck_timeout(void)
{
  return g_LINKCHECK_TIMEOUT;
}

/**
 * @brief set linkcheck_timeout
 */
bool set_linkcheck_timeout(uint32_t timeout)
{
  if (api.system.flash.set(LINKCHECK_TIMEOUT_OFFSET, (uint8_t*)&timeout, 4) == false) {
    return false;
  }
  g_LINKCHECK_TIMEOUT = timeout;
  return true;
}

/**
 * @brief handle linkcheck_timeout
 */
int linkcheck_timeout_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_linkcheck_timeout());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint32_t tmp = strtoul(param->argv[0], NULL, 10);
    if (tmp > 1440) {
      return AT_PARAM_ERROR;
    }

    if (set_linkcheck_timeout(tmp) == false) {
      return AT_ERROR;
    }
    reset_linkcheck_start();
    reset_last_send_ok_time();
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for linkcheck_timeout
 */
void register_linkcheck_timeout_atcmd(void)
{
  api.system.atMode.add("LCTIMEOUT", "Set/Get the linkcheck timeout for re-join (0-1440 minutes, 0: Disable)", "JOIN", linkcheck_timeout_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize linkcheck_timeout
 */
void init_linkcheck_timeout()
{
  if (api.system.flash.get(LINKCHECK_TIMEOUT_OFFSET, (uint8_t*)&g_LINKCHECK_TIMEOUT, 4) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  else
  {
    if (g_LINKCHECK_TIMEOUT == 0xFFFFFFFF)
    {
      set_linkcheck_timeout(DEFAULT_LINKCHECK_TIMEOUT);
    }
  }
  register_linkcheck_timeout_atcmd();
}


//md_threshold, md_duration, md_number, md_silent_period
uint8_t   g_md_enable;        //Motion detect, enable
uint16_t  g_md_threshold;     //Motion detect, detect threshold
uint16_t  g_md_sample_rate;   //Motion detect, lis3dh sample
uint16_t  g_md_duration;      //Motion detect, the duration for event trigger
uint8_t  g_md_number;         //Motion detect, the number of deteted for event trigger
uint16_t  g_md_silent_period; //Motion detect, the silent period after event trigger

/**
 * @brief get motion detect enable
 */
uint8_t get_md_enable(void)
{
  return g_md_enable;
}

/**
 * @brief set motion detect enable
 */
bool set_md_enable(uint8_t enable)
{
  if (api.system.flash.set(MD_ENABLE_OFFSET, &enable, 1) == false) {
    return false;
  }
  g_md_enable = enable;
  return true;
}

/**
 * @brief get motion detect threshold
 */
uint16_t get_md_threshold(void)
{
  return g_md_threshold;
}

/**
 * @brief set motion detect threshold
 */
bool set_md_threshold(uint16_t threshold)
{
  if (api.system.flash.set(MD_THRESHOLD_OFFSET, (uint8_t*)&threshold, 2) == false) {
    return false;
  }
  g_md_threshold = threshold;
  return true;
}

/**
 * @brief get lis3dh sample rate
 */
uint16_t get_md_sample_rate(void)
{
  return g_md_sample_rate;
}

/**
 * @brief set lis3dh sample rate
 */
bool set_md_sample_rate(uint16_t sample_rate)
{
  if (api.system.flash.set(MD_SAMPLE_RATE_OFFSET, (uint8_t*)&sample_rate, 2) == false) {
    return false;
  }
  g_md_sample_rate = sample_rate;
  return true;
}

/**
 * @brief get motion detect duration
 */
uint16_t get_md_duration(void)
{
  return g_md_duration;
}

/**
 * @brief set motion detect duration
 */
bool set_md_duration(uint16_t duration)
{
  if (api.system.flash.set(MD_DURATION_OFFSET, (uint8_t*)&duration, 2) == false) {
    return false;
  }
  g_md_duration = duration;
  return true;
}

/**
 * @brief get motion detect number
 */
uint8_t get_md_number(void)
{
  return g_md_number;
}

/**
 * @brief set motion detect number
 */
bool set_md_number(uint8_t number)
{
  if (api.system.flash.set(MD_NUMBER_OFFSET, (uint8_t*)&number, 1) == false) {
    return false;
  }
  g_md_number = number;
  return true;
}

/**
 * @brief get motion detect silent period
 */
uint16_t get_md_silent_period(void)
{
  return g_md_silent_period;
}

/**
 * @brief set motion detect silent period
 */
bool set_md_silent_period(uint16_t silent_period)
{
  if (api.system.flash.set(MD_SILENT_PERIOD_OFFSET, (uint8_t*)&silent_period, 2) == false) {
    return false;
  }
  g_md_silent_period = silent_period;
  return true;
}

/**
 * @brief the handler of ATC Command for motion detect enable
 */
int md_enable_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_enable());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint8_t tmp = (uint8_t)strtoul(param->argv[0], NULL, 10);
    if (tmp > 1) {
      return AT_PARAM_ERROR;
    }

    if (get_md_enable() == tmp) {
      return AT_OK;
    }

    if (set_md_enable(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_begin();
      }
      else {
        lis3dh_md_end();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for motion detect status
 */
void register_md_enable_atcmd(void)
{
  api.system.atMode.add("MDEN", "Set/Get the Motion Detection status (0: Disable, 1: Enable)", "MD", md_enable_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief the handler of ATC Command for motion detect threshold
 */
int md_threshold_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_threshold());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint16_t tmp = (uint16_t)strtoul(param->argv[0], NULL, 10);
    if(tmp < 50 || tmp > 16000)
      return AT_PARAM_ERROR;

    if (get_md_threshold() == tmp) {
      return AT_OK;
    }

    if (set_md_threshold(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_pause();
        lis3dh_md_resume();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for motion detect threshold
 */
void register_md_threshold_atcmd(void)
{
  api.system.atMode.add("MDTH", "Set/Get the detection threshold of the accelerometer (50~16000 mG)", "MD", md_threshold_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief the handler of ATC Command for lis3dh sample rate
 */
int md_sample_rate_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_sample_rate());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint16_t tmp = (uint16_t)strtoul(param->argv[0], NULL, 10);
    if(tmp > 1)
      return AT_PARAM_ERROR;

    if (get_md_sample_rate() == tmp) {
      return AT_OK;
    }

    if (set_md_sample_rate(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_pause();
        lis3dh_md_resume();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for lis3dh sample rate
 */
void register_md_sample_rate_atcmd(void)
{
  api.system.atMode.add("MDSR", "Set/Get the sample rate of the accelerometer (0: 1 Hz, 1: 10 Hz)", "MD", md_sample_rate_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief the handler of ATC Command for motion detect duration
 */
int md_duration_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_duration());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint16_t tmp = (uint16_t)strtoul(param->argv[0], NULL, 10);
    if(tmp < 300 || tmp > 15000)
      return AT_PARAM_ERROR;

    if (get_md_duration() == tmp) {
      return AT_OK;
    }

    if (set_md_duration(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_pause();
        lis3dh_md_resume();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for motion detect duration
 */
void register_md_duration_atcmd(void)
{
  api.system.atMode.add("MDDUR", "Set/Get the detect duration of the Motion Detection (300~15000 mseconds)", "MD", md_duration_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief the handler of ATC Command for motion detect number
 */
int md_number_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_number());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint8_t tmp = (uint8_t)strtoul(param->argv[0], NULL, 10);
    if (tmp < 1 || tmp > 10) {
      return AT_PARAM_ERROR;
    }

    if (get_md_number() == tmp) {
      return AT_OK;
    }

    if (set_md_number(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_pause();
        lis3dh_md_resume();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for motion detect number
 */
void register_md_number_atcmd(void)
{
  api.system.atMode.add("MDNUM", "Set/Get the number of triggers required to send an event uplink (1~10)", "MD", md_number_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief the handler of ATC Command for motion detect silent period
 */
int md_silent_period_handle(SERIAL_PORT port, char *cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(get_md_silent_period());
  } else if (param->argc == 1) {
    if (!isdigit(*(param->argv[0]))) {
      return AT_PARAM_ERROR;
    }

    uint16_t tmp = (uint16_t)strtoul(param->argv[0], NULL, 10);
    if(tmp < 10 || tmp > 3600)
      return AT_PARAM_ERROR;

    if (get_md_silent_period() == tmp) {
      return AT_OK;
    }

    if (set_md_silent_period(tmp) == false) {
      return AT_ERROR;
    }
    else {
      if (get_md_enable()) {
        lis3dh_md_pause();
        lis3dh_md_resume();
      }
    }
  }
  else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

/**
 * @brief Register ATC Command for motion detect silent period
 */
void register_md_silent_period_atcmd(void)
{
  api.system.atMode.add("MDSP", "Set/Get the silent period after sending an event uplink (10~3600 seconds)", "MD", md_silent_period_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
}

/**
 * @brief Initialize the parameters for motion detect
 */
void init_md_atcmd()
{
  if (api.system.flash.get(MD_ENABLE_OFFSET, &g_md_enable, 1) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_enable == 0xFF) {
    g_md_enable = DEFAULT_MD_ENABLE;
    set_md_enable(g_md_enable);
  }
  //Serial.printf("md_enable: %d(%d)\r\n", g_md_enable, get_md_enable());

  if (api.system.flash.get(MD_THRESHOLD_OFFSET, (uint8_t*)&g_md_threshold, 2) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_threshold == 0xFFFF) {
    g_md_threshold = DEFAULT_MD_THRESHOLD;
    set_md_threshold(g_md_threshold);
  }
  //Serial.printf("md_threshold: %d(%d)\r\n", g_md_threshold, get_md_threshold());

  if (api.system.flash.get(MD_SAMPLE_RATE_OFFSET, (uint8_t*)&g_md_sample_rate, 2) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_sample_rate == 0xFFFF) {
    g_md_sample_rate = DEFAULT_MD_SAMPLE_RATE;
    set_md_sample_rate(g_md_sample_rate);
  }
  //Serial.printf("g_md_sample_rate: %d(%d)\r\n", g_md_sample_rate, get_md_sample_rate());

  if (api.system.flash.get(MD_DURATION_OFFSET, (uint8_t*)&g_md_duration, 2) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_duration == 0xFFFF) {
    g_md_duration = DEFAULT_MD_DURATION;
    set_md_duration(g_md_duration);
  }
  //Serial.printf("md_duration: %d(%d)\r\n", g_md_duration, get_md_duration());

  if (api.system.flash.get(MD_NUMBER_OFFSET, (uint8_t*)&g_md_number, 1) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_number == 0xFF) {
    g_md_number = DEFAULT_MD_NUMBER;
    set_md_number(g_md_number);
  }
  //Serial.printf("md_number: %d(%d)\r\n", g_md_number, get_md_number());

  if (api.system.flash.get(MD_SILENT_PERIOD_OFFSET, (uint8_t*)&g_md_silent_period, 2) == false) {
    Serial.print("Reading flash data failure...\r\n");
  }
  if (g_md_silent_period == 0xFFFF) {
    g_md_silent_period = DEFAULT_MD_SILENT_PERIOD;
    set_md_silent_period(g_md_silent_period);
  }
  //Serial.printf("md_silent_period: %d(%d)\r\n", g_md_silent_period, get_md_silent_period());

  register_md_enable_atcmd();
  register_md_threshold_atcmd();
  register_md_sample_rate_atcmd();
  register_md_duration_atcmd();
  register_md_number_atcmd();
  register_md_silent_period_atcmd();
}

