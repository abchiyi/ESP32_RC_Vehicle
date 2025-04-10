
#include "RZ_series.h"

RZ_HBridgeDriver::RZ_HBridgeDriver(uint8_t fi, uint8_t bi)
{
    this->pin_forward = fi;
    this->pin_backward = bi;
    pinMode(pin_forward, OUTPUT);
    pinMode(pin_backward, OUTPUT);
}

void RZ_HBridgeDriver::forward(uint8_t speed)
{
    analogWrite(pin_forward, speed);
    analogWrite(pin_backward, 0);
}

void RZ_HBridgeDriver::backward(uint8_t speed)
{
    analogWrite(pin_backward, speed);
    analogWrite(pin_forward, 0);
}

void RZ_HBridgeDriver::stop()
{
    analogWrite(pin_forward, 255);
    analogWrite(pin_backward, 255);
}

void RZ_HBridgeDriver::free()
{
    analogWrite(pin_forward, 0);
    analogWrite(pin_backward, 0);
}
