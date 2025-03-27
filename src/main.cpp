#include <Arduino.h>
// #include "esp_crtp.h"
#include "WiFi.h"
#include <Wire.h>
#include "wifiLink.h"
#include "radio.h"

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9);

  wifi_init();

  vTaskDelete(NULL); // KILL MAIN LOOP
}

void loop()
{
}
