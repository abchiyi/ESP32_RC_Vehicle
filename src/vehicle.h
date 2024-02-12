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
};

void updateRecvCB(const uint8_t *incomingData);

class Vehicle
{
private:
  bool HOLD_RT = false;
  bool HOLD_LT = false;
  bool changed = false;

public:
  static int ang;

  void begin(bool *connected);
  void update();
};
