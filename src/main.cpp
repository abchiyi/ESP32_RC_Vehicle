#include <Arduino.h>
#include <Radio.h>

#define CHANNEL 1

Radio radio;

void setup()
{
  Serial.begin(115200);

  radio.begin();
  radio.startPairing();

  // esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  // Chill
}