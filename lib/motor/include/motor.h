#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"

class motor
{
private:
    /* data */
public:
    virtual void forward(uint8_t) = 0;  // 前进
    virtual void backward(uint8_t) = 0; // 后退
    virtual void stop() = 0;            // 带力度控制的刹车
    virtual void free() = 0;            // 滑行
};

#endif
