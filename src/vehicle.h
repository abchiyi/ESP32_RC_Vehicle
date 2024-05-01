#include <Arduino.h>

#define REVERSE -1
#define FORWARD 1
#define BRAKE 0
#define SLIDE 2

#define BRAKE_RT 1
#define BRAKE_LT 0

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

void updateRecvCB(uint8_t *incomingData);

class Vehicle
{
private:
public:
  static int ang;
  static int gear;

  void begin(bool *connected);
  void update();
};
