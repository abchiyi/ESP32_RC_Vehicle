#include <Arduino.h>

// 挡位状态
typedef enum gear
{
  SLIDE,
  BRAKE,
  REVERSE,
  FORWARD
} gear_t;

#define BRAKE_RT 1
#define BRAKE_LT 0

class Vehicle
{
private:
public:
  void begin();
};
