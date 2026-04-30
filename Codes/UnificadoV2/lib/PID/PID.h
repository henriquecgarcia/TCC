#pragma once
#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd, float dt);
  void setTunings(float kp, float ki, float kd);
  void setLimits(float minOut, float maxOut);
  float compute(float setpoint, float measurement);
  void reset();

private:
  float _kp;
  float _ki;
  float _kd;
  float _dt;
  float _integral;
  float _prevError;
  float _minOut;
  float _maxOut;
  static float clamp(float v, float lo, float hi);
};
