#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel);
  void begin(uint16_t pwmFreq, uint8_t pwmResolutionBits);
  void setPWM(int16_t pwm);
  void brake();
  void coast();

private:
  uint8_t _in1;
  uint8_t _in2;
  uint8_t _pwmPin;
  uint8_t _pwmChannel;
  uint8_t _maxPwm;
};
