// #include <esp32-hal-log.h>
#include <esp_log.h>
#include <Arduino.h>
#include <vehicle.h>
#include <Radio.h>

#define TAG "Main RC Vehicle"
#define SSID "Slave_2"
#define CHANNEL 1

Vehicle vehicle;
Radio radio;

void taskReadBatteryVolt(void *pt)
{
  int analogVolts;
  const int pin = 36;
  analogReadResolution(12);
  int conter = 0;
  while (true)
  {
    conter++; // 每10个循环返回写入一次平均电压
    analogVolts += analogReadMilliVolts(pin);
    if (conter == 20)
    {
      conter = 0;
      radio.SendData.volts = (float)(analogVolts / 20 * 3) / 1000.0;
      analogVolts = 0;
    }
    vTaskDelay(5);
  }
}

void setup()
{
  Serial.begin(115200);
  radio.begin(SSID, CHANNEL, updateRecvCB);
  vehicle.begin(&radio.connected);
  xTaskCreate(taskReadBatteryVolt, "taskReadBatteryVolt", 4096, NULL, 2, NULL);
}

void loop()
{
  vehicle.update();
}