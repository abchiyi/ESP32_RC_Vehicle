#include <Arduino.h>
// #include "esp_crtp.h"
#include "WiFi.h"
#include <Wire.h>
#include "wifiLink.h"

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9);
  vTaskDelete(NULL); // KILL MAIN LOOP
}

void loop()
{
}
