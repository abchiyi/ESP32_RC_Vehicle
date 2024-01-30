// #include <esp32-hal-log.h>
#include <esp_log.h>
#include <Arduino.h>
#include <Radio.h>

#define TAG "Main RC Vehicle"
#define SSID "Slave_2"
#define CHANNEL 1

Radio radio;

void setup()
{
  radio.begin(SSID, CHANNEL);
}

void loop()
{
  Serial.println(
      radio.controller().trigLT);
}