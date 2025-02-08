#include <tool.h>
#include <INA226.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

void nvs_call(const char *name_space, std::function<void(Preferences &)> cb_fn)
{
  Preferences PREFS;
  PREFS.begin(name_space);
  cb_fn(PREFS);
  PREFS.end();
}

#define TAG_INA226 "INA226"                           // log 标签
#define MAX_CURREN(shunt) (81.92f / ((shunt) * 1000)) // 最大电流计算
#define SHUNT 0.005                                   // 检流电阻值
#define INA226_ADDR 0x40                              // 设备地址
TimerHandle_t TIMER_POWER_CHECK;                      // 任务定时器

INA226 ina226(INA226_ADDR);

void cb_fn_ina226(TimerHandle_t xTimer)
{
  ESP_LOGI("POWER", "VBUS %.2fV, SHUT %.2fuV, CUR %.2fA, POW %.2fuW",
           ina226.getBusVoltage(),
           ina226.getShuntVoltage_uV(),
           ina226.getCurrent(),
           ina226.getPower_uW());
}

/**
 * @brief 启动功率传感器
 */
void setPowerCheck()
{
  auto max_curren = MAX_CURREN(SHUNT);

  TIMER_POWER_CHECK = xTimerCreate("power check",
                                   pdMS_TO_TICKS(1000),
                                   pdTRUE,
                                   NULL,
                                   cb_fn_ina226);

  if (TIMER_POWER_CHECK == nullptr)
  {
    ESP_LOGE(TAG_INA226, "Create timer FAIL");
    return;
  };

  Wire.begin();
  if (!ina226.begin())
  {
    ESP_LOGE(TAG_INA226, "could not connect. Fix and Reboot");
  };

  switch (ina226.setMaxCurrentShunt(max_curren, SHUNT))
  {
  case INA226_ERR_SHUNTVOLTAGE_HIGH:
    ESP_LOGE(TAG_INA226, "INA226_ERR_SHUNTVOLTAGE_HIGH,MAX_CURREN %.2f,SHUNT %.3f",
             max_curren,
             SHUNT);
    break;
  case INA226_ERR_MAXCURRENT_LOW:
    ESP_LOGE(TAG_INA226, "INA226_ERR_MAXCURRENT_LOW,MAX_CURREN %d.2f,SHUNT %d.3f",
             max_curren,
             SHUNT);
    break;
  case INA226_ERR_SHUNT_LOW:
    ESP_LOGE(TAG_INA226, "INA226_ERR_SHUNT_LOW,MAX_CURREN %d.2f,SHUNT %.3f",
             max_curren,
             SHUNT);
    break;

  default:
    if (xTimerStart(TIMER_POWER_CHECK, 10) == pdFAIL)
      esp_system_abort("timer start fail");

    ESP_LOGI(TAG_INA226, "Power check set ok");
    break;
  }
}
