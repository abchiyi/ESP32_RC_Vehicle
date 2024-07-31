#include <ESP32Servo.h>
#include <vehicle.h>
#include <esp_log.h>
#include <Radio.h>
#include "math.h"
#include "pins.h"

#define TAG "vehicle"

Servo TurnServo;
bool *ControllerConnected; // 连接状态

#define PWM_RESOLUTION 10                                 // pwm 分辨率
#define PWM_DUTY_MAX int(std::pow(2, PWM_RESOLUTION) - 1) // pwm 最大占空比

// PWM 通道
#define CHANNEL_MOVE_F 2          // 马达驱动F
#define CHANNEL_MOVE_R 3          // 马达驱动R
#define CHANNEL_LIGHT_L NULL      // 左转向灯
#define CHANNEL_LIGHT_R NULL      // 右转向灯
#define CHANNEL_HEADLIGHT NULL    // 大灯
#define CHANNEL_STOPLIGHT 4       // 刹车灯
#define CHANNEL_STATUSLIGHT NULL  // 状态灯
#define CHANNEL_REVERSING_LIGHT 5 // 倒车灯

bool DISTANT_LIGHT = false; // 远光
bool LOW_BEAM = false;      // 近光
bool WIDTH_LAMP = false;    // 示宽灯
int REVERSING_LIGHT = 0;    // 倒车灯
int HeadLight = 0;          // 大灯   亮度0~255
int StopLight = 0;          // 刹车灯 亮度0~255
bool brake = false;         // 刹车
bool LightTurnL = false;    // 左转灯
bool LightTurnR = false;    // 右转灯
bool HazardLight = false;   // 危险报警灯

const double angStep = 90.00 / 256.00; // 转向°步长

/* 设置 pwm 输出引脚 */
void setPWMPin(int pin, int pwmChannel)
{
  pinMode(pin, OUTPUT);
  ledcSetup(pwmChannel, 30000, 10); // 10bit 0 ~ 1023;
  ledcAttachPin(pin, pwmChannel);
  ledcWrite(pwmChannel, 0);
}

/* 状态灯任务 */
void TaskStatusLight(void *pt)
{
  int pwmChannel = CHANNEL_STATUSLIGHT;
  int resolution = 100;
  double step = (double)255 / (double)resolution;
  while (true)
  {
    if (*ControllerConnected)
    {
      ledcWrite(pwmChannel, 255);
      vTaskDelay(100);
      ledcWrite(pwmChannel, 0);
      vTaskDelay(100);
      ledcWrite(pwmChannel, 255);
      vTaskDelay(100);
      ledcWrite(pwmChannel, 0);
      vTaskDelay(2000);
    }
    else
    {
      for (int i = 0; i <= 100; i = i + 5)
      {
        ledcWrite(pwmChannel, round((double)i * step));
        vTaskDelay(3);
      }
      vTaskDelay(100);

      for (int i = 100; i >= 0; i = i - 5)
      {
        ledcWrite(pwmChannel, round((double)i * step));
        vTaskDelay(3);
      }
      vTaskDelay(100);
    }
  }
}

/* 转向灯任务 */
void TaskIndicatorLight(void *pt)
{
  bool increasing = false;
  int lightLevel = 100;

  const int MAX = 100;
  const int MIN = 0;
  const double step = (double)255 / (double)MAX;

  while (true)
  {

    if (LightTurnL || LightTurnR || HazardLight)
    {
      increasing ? lightLevel++ : lightLevel--;
      increasing = lightLevel >= MAX   ? false
                   : lightLevel <= MIN ? true
                                       : increasing;
    }

    int v = lightLevel <= MIN ? MIN : round((double)lightLevel * step);
    LightTurnL || HazardLight // 左转向灯
        ? ledcWrite(CHANNEL_LIGHT_L, v)
        : ledcWrite(CHANNEL_LIGHT_L, 0);

    LightTurnR || HazardLight // 右转向灯
        ? ledcWrite(CHANNEL_LIGHT_R, v)
        : ledcWrite(CHANNEL_LIGHT_R, 0);

    lightLevel == MIN || lightLevel == MAX ? vTaskDelay(200) : vTaskDelay(1);
  }
}

// 车辆控制主任务
void task_vehicle_main(radio_data_t radio_data)
{

  auto set_servo = [&]()
  {
    auto data = read_channel_data(radio_data, 1, true);

    // XXX 缩减转向比
    int v = (round((double)abs(data.value) / (double)8) * angStep) * 0.4; // 计算转向角度
    // int v = (round((double)abs(data.value) / (double)8) * angStep) * 0.5; // 计算转向角度
    auto ang = data.value < 0 ? 90 + v : data.value > 0 ? 90 - v
                                                        : 90;

    // ang = 180 - ang; // 对输出结果取反
    TurnServo.write(ang);
    // ESP_LOGI(TAG, "ang %d, v: %d", ang, data.value);
  };

  auto set_motor = [&]()
  {
    auto data = read_channel_data(radio_data, 0, true);

    auto gear = (radio_data.channel[0] >> 2) & 0x03;
    brake = radio_data.channel[0] & 0x03;

    switch (gear) // 读取挡位
    {
    case REVERSE:
      ledcWrite(CHANNEL_MOVE_R, data.value);
      ledcWrite(CHANNEL_MOVE_F, 0);
      break;

    case FORWARD:
      ledcWrite(CHANNEL_MOVE_F, data.value);
      ledcWrite(CHANNEL_MOVE_R, 0);
      break;

    case SLIDE:
      ledcWrite(CHANNEL_MOVE_R, 0);
      ledcWrite(CHANNEL_MOVE_F, 0);
      break;

    case BRAKE:
      ledcWrite(CHANNEL_MOVE_F, PWM_DUTY_MAX);
      ledcWrite(CHANNEL_MOVE_R, PWM_DUTY_MAX);
      break;
    }

    if (brake)
    {
      ledcWrite(CHANNEL_MOVE_F, PWM_DUTY_MAX);
      ledcWrite(CHANNEL_MOVE_R, PWM_DUTY_MAX);
    }

    // 倒车灯
    ledcWrite(CHANNEL_REVERSING_LIGHT, gear == REVERSE ? 1023 : 0);
    // 刹车灯
    ledcWrite(CHANNEL_STOPLIGHT, brake ? 1023 : 0);

    // ESP_LOGI(TAG, "g %d, v %d, b : %d", gear, data.value, brake);
  };
  set_motor();
  set_servo();
}

/* 设置灯光任务 */
void LightSetup()
{

  // xTaskCreate(TaskLight, "TaskHeadlight", 1024, NULL, 3, NULL);
  // xTaskCreate(TaskStatusLight, "TaskStatusLight", 1024, NULL, 3, NULL);
  // xTaskCreate(TaskLightControll, "TaskHeadlightControll", 1024, NULL, 3, NULL);
  xTaskCreate(TaskIndicatorLight, "Indicator Light", 1024, NULL, 3, NULL);
  // xTaskCreate(TaskIndicatorLightControl, "IndicatorLightControl", 1024, NULL, 3, NULL);
}

void Vehicle::begin()
{

  radio.onDisconnect = [&]()
  {
    ESP_LOGI(TAG, "RADIO_DISCONNECT STOP !!!!!");
    TurnServo.write(90);
    ledcWrite(CHANNEL_MOVE_F, 0);
    ledcWrite(CHANNEL_MOVE_R, 0);
  };

  radio.__onRecv = task_vehicle_main;

  LightSetup();

  // 设置舵机
  pinMode(PIN_SERVO, OUTPUT);
  TurnServo.setPeriodHertz(50);
  TurnServo.attach(PIN_SERVO, 50, 2500);

  // Motor PIN
  setPWMPin(PIN_MOTOR_F, CHANNEL_MOVE_F);
  setPWMPin(PIN_MOTOR_R, CHANNEL_MOVE_R);

  // int count = 35;
  // for (size_t i = 0; i < count; i++)
  // {
  //   ledcSetup(CHANNEL_MOVE_F, i * 100, 10);
  //   ledcWrite(CHANNEL_MOVE_F, 50);
  //   vTaskDelay(50);
  //   ledcWrite(CHANNEL_MOVE_F, 0);
  //   ledcWrite(CHANNEL_MOVE_R, 0);
  //   vTaskDelay(100);
  //   ESP_LOGI(TAG, "%d", i);
  // }
  // setPWMPin(PIN_MOTOR_F, CHANNEL_MOVE_F);

  // delay(100);
  // ledcWrite(CHANNEL_MOVE_F, 0);
  // delay(100);
  // ledcWrite(CHANNEL_MOVE_F, 255);
  // delay(100);

  // setPWMPin(PIN_R_LIGHT, CHANNEL_LIGHT_R);
  // setPWMPin(PIN_L_LIGHT, CHANNEL_LIGHT_L);
  // setPWMPin(PIN_HEADLIGHT, CHANNEL_HEADLIGHT);
  // setPWMPin(PIN_STATUSLIGHT, CHANNEL_STATUSLIGHT);
  // setPWMPin(PIN_STOPLIGHT, CHANNEL_STOPLIGHT);
  // setPWMPin(PIN_REVERSING_LIGHT, CHANNEL_REVERSING_LIGHT);

  // xTaskCreate(task_vehicle_main, "Vehicle main task", 1024 * 4, NULL, 1, NULL);
}
