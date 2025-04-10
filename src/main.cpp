#include <Arduino.h>
#include "radio.h"
#include "wifiLink.h"
#include "Esp32Servo.h"
#include "RZ_series.h"

RZ_HBridgeDriver motor(0, 1);

Servo servo;

void cb_setpoint(CRTPPacket *pkt)
{
  float ROLL;
  float PITCH;
  float YAW;
  uint16_t THRUST;

  memcpy(&ROLL, &pkt->data[0], sizeof(float));
  memcpy(&PITCH, &pkt->data[4], sizeof(float));
  memcpy(&YAW, &pkt->data[8], sizeof(float));
  memcpy(&THRUST, &pkt->data[12], sizeof(uint16_t));

  // 180° Servo
  servo.write(90 - YAW);

  if (PITCH < 0)
    motor.backward(-PITCH);
  else if (PITCH > 0)
    motor.forward(PITCH);
  else
    motor.stop();

  ESP_LOGI("CRTP", "ROLL: %.2f, PITCH: %.2f, YAW: %.2f, THRUST: %d",
           ROLL, PITCH, YAW, THRUST);
};

void setup()
{
  Serial.begin(115200);
  init_radio(WiFiGetLink());
  radio_set_port_callback(CRTP_PORT_SETPOINT, cb_setpoint);

  if (!servo.attached())
  {
    servo.setPeriodHertz(50); // standard 50 hz servo
    servo.attach(3, 1000, 2000);
  }

  servo.write(90);

  vTaskDelete(NULL);
}

void loop()
{
}
