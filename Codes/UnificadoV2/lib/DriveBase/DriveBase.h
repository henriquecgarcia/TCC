#pragma once
#include <Arduino.h>
#include "../MotorDriver/MotorDriver.h"
#include "../Encoder/Encoder.h"
#include "../PID/PID.h"
#include "../../constants/RobotConfig.h"
#include "../../src/SharedTypes.h"

class DriveBase {
public:
  DriveBase(MotorDriver& leftMotor, MotorDriver& rightMotor, Encoder& leftEncoder, Encoder& rightEncoder);
  void begin();
  WheelSample update(float targetLeftMps, float targetRightMps, float dt);
  void stop();

private:
  MotorDriver& _leftMotor;
  MotorDriver& _rightMotor;
  Encoder& _leftEncoder;
  Encoder& _rightEncoder;
  PID _leftPid;
  PID _rightPid;
  float speedFromTicks(long ticks, float dt) const;
};
