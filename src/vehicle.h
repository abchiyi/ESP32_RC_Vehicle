#include <Arduino.h>

#define REVERSE -1
#define FORWARD 1
#define BRAKE 0
#define SLIDE 2

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
public:
  static int ang;

  void begin(bool *connected);
  void update();
};
