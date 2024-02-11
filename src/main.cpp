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
void setup()
{
  Serial.begin(115200);
  radio.begin(SSID, CHANNEL, updateRecvCB);
  vehicle.begin(&radio.connected);
}

void loop()
{
  // Serial.println(Controller.trigLT);
}