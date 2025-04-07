
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
    digitalWrite(pin_backward, LOW);
}

void RZ_HBridgeDriver::backward(uint8_t speed)
{
    analogWrite(pin_backward, speed);
    digitalWrite(pin_forward, LOW);
}

void RZ_HBridgeDriver::stop(uint8_t speed)
{
    digitalWrite(pin_forward, HIGH);
    digitalWrite(pin_backward, HIGH);
}

void RZ_HBridgeDriver::free()
{
    digitalWrite(pin_forward, LOW);
    digitalWrite(pin_backward, LOW);
}
