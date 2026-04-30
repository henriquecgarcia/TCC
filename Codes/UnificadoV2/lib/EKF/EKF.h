#pragma once
#include <Arduino.h>
#include "../../src/SharedTypes.h"

class EKF {
public:
  EKF();
  void reset(float x, float y, float theta);
  void predict(float vLeft, float vRight, float dt);
  void updateTheta(float measuredTheta);
  Pose2D pose() const;

private:
  Pose2D _x;
  float _p[3][3];
  float _q[3];
  float _rTheta;
};
