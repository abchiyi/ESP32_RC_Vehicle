#ifndef RZ_series_h
#define RZ_series_h

#include "Arduino.h"
#include "motor.h"

class RZ_HBridgeDriver : public motor
{
private:
    uint8_t pin_forward, pin_backward;

public:
    RZ_HBridgeDriver(uint8_t fi, uint8_t bi);
    // ~RZ_series();

    void forward(uint8_t) override;
    void backward(uint8_t) override;
    void stop(uint8_t) override;
    void free() override;
};

#endif
