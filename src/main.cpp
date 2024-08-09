#include <esp_log.h>
#include <Arduino.h>
#include <vehicle.h>
#include <Radio.h>
#include "tool.h"

#define TAG "Main RC Vehicle"
#define SSID "Slave"
#define CHANNEL 1

Vehicle vehicle;

// void ISR()
// {
//   if (radio.status == RADIO_CONNECTED)
//     radio.status = RADIO_BEFORE_DISCONNECT;
//   else
//     radio.status = RADIO_BEFORE_WAIT_CONNECTION;
// };

#define BUTTON_BOOT 9

void setup()
{

  Serial.begin(115200);

  vehicle.begin();
  radio.begin(SSID, CHANNEL);
  setPowerCheck();

  // attachInterrupt(digitalPinToInterrupt(BUTTON_BOOT), ISR, RISING);
}

void loop()
{
}