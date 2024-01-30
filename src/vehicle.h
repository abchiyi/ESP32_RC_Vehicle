#include <Arduino.h>

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
} Controller;

// 回调函数，处理radio接收到的数据
void updateRecvCB(const uint8_t *incomingData)
{
  memcpy(&Controller, incomingData, sizeof(Controller));
}

class Vehicle
{
private:
public:
  void begin();
};
