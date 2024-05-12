#include <Arduino.h>

// 挡位状态
typedef enum
{
  R, // REVERSE
  D, // FORWARD
  N, // SLIDE
  B, // BRAKE
} gear_t;

#define BRAKE_RT 1
#define BRAKE_LT 0

class Vehicle
{
private:
public:
  void begin();
};
