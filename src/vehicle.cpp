#include <ESP32Servo.h>
#include <vehicle.h>
#include <esp_log.h>
#include <Radio.h>

#define TAG "vehicle"

Servo TurnServo;
bool *ControllerConnected; // 连接状态

// 定义控制 Pin
#define PIN_MOVE_F 10    // 前进控制
#define PIN_MOVE_R 16    // 倒车控制
#define CHANNEL_MOVE_F 4 // 马达驱动 pwm 通道
#define CHANNEL_MOVE_R 5 // 马达驱动 pwm 通道
#define PIN_TURN 18      // 转向控制

#define PIN_HEADLIGHT 19       // 大灯
#define PIN_STOPLIGHT 27       // 刹车灯
#define PIN_STATUSLIGHT 23     // 状态灯
#define PIN_REVERSING_LIGHT 26 // 倒车灯
#define PIN_L_LIGHT 33         // 左转向灯
#define PIN_R_LIGHT 25         // 右转向灯

// PWM 通道
#define CHANNEL_LIGHT_L 2         // 左转向灯
#define CHANNEL_LIGHT_R 3         // 右转向灯
#define CHANNEL_HEADLIGHT 6       // 大灯PWM通道
#define CHANNEL_STOPLIGHT 7       // 刹车灯PWM通道
#define CHANNEL_STATUSLIGHT 1     // 状态灯
#define CHANNEL_REVERSING_LIGHT 8 // 倒车灯PWM通道

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
  ledcSetup(pwmChannel, 2000, 8);
  ledcAttachPin(pin, pwmChannel);
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

// 更新马达状态
void updateMotor(radio_data_t *data)
{

  gear_t gea = N;

  switch (gea)
  {
  case B:
    ledcWrite(CHANNEL_MOVE_F, 255);
    ledcWrite(CHANNEL_MOVE_R, 255);
    break;

  case D:
    // ledcWrite(CHANNEL_MOVE_F, (int)(RT / 4));
    ledcWrite(CHANNEL_MOVE_R, 0);
    break;

  case R:
    // ledcWrite(CHANNEL_MOVE_R, (int)(LT / 4));
    ledcWrite(CHANNEL_MOVE_F, 0);
    break;

  case N: // SLIDE
    ledcWrite(CHANNEL_MOVE_R, 0);
    ledcWrite(CHANNEL_MOVE_F, 0);
    break;
  }
}

// 车辆控制主任务
void task_vehicle_main(void *pd)
{
  radio_data_t data;
  while (true)
  {
    if (radio.get_data(&data) == ESP_OK)
    {
      // continue; // 未获取到数据时跳过循环
      digitalWrite(PIN_MOVE_F, data.channel[0]);
      // updateMotor(&data);
    }
    vTaskDelay(1);
  }
}

void Vehicle::begin()
{
  // LightSetup();

  // 设置电调
  // setPWMPin(PIN_MOVE_F, CHANNEL_MOVE_F); // input 1/3
  // setPWMPin(PIN_MOVE_R, CHANNEL_MOVE_R); // input 2/4
  pinMode(PIN_MOVE_F, OUTPUT);

  // 设置舵机
  pinMode(PIN_TURN, OUTPUT);
  TurnServo.setPeriodHertz(50);
  TurnServo.attach(PIN_TURN, 50, 2500);

  xTaskCreate(task_vehicle_main, "Vehicle main task", 1024 * 4, NULL, 1, NULL);
}
