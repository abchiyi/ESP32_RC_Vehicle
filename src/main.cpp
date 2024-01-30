// #include <esp32-hal-log.h>
#include <esp_log.h>
#include <Arduino.h>
#include <Radio.h>

#define TAG "Main RC Vehicle"
#define SSID "Slave_2"
#define CHANNEL 1

Radio radio;
struct ControllerStatus
{
  bool btnA, btnB, btnX, btnY;
  bool btnShare, btnStart, btnSelect, btnXbox;
  bool btnLB, btnRB;
  bool btnLS, btnRS;
  bool btnDirUp, btnDirLeft, btnDirRight, btnDirDown;
  int16_t joyLHori;
  int16_t joyLVert;
  int16_t joyRHori;
  int16_t joyRVert;
  int16_t trigLT, trigRT;
} Data;

void recvCB(const uint8_t *incomingData)
{
  memcpy(&Data, incomingData, sizeof(Data));
}

void setup()
{
  Serial.begin(115200);
  radio.begin(SSID, CHANNEL, recvCB);
}

void loop()
{
  Serial.println(Data.trigLT);
}