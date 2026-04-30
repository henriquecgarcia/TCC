#include "DriveBase.h"

DriveBase::DriveBase(MotorDriver& leftMotor, MotorDriver& rightMotor, Encoder& leftEncoder, Encoder& rightEncoder)
  : _leftMotor(leftMotor), _rightMotor(rightMotor), _leftEncoder(leftEncoder), _rightEncoder(rightEncoder),
    _leftPid(420.0f, 55.0f, 4.0f, RobotConfig::CONTROL_PERIOD_MS / 1000.0f),
    _rightPid(420.0f, 55.0f, 4.0f, RobotConfig::CONTROL_PERIOD_MS / 1000.0f) {}

void DriveBase::begin() {
  _leftMotor.begin(RobotConfig::PWM_FREQ, RobotConfig::PWM_RES_BITS);
  _rightMotor.begin(RobotConfig::PWM_FREQ, RobotConfig::PWM_RES_BITS);
  _leftEncoder.begin();
  _rightEncoder.begin();
  _leftPid.setLimits(-255.0f, 255.0f);
  _rightPid.setLimits(-255.0f, 255.0f);
}

WheelSample DriveBase::update(float targetLeftMps, float targetRightMps, float dt) {
  const long leftDelta = _leftEncoder.readAndResetDelta();
  const long rightDelta = _rightEncoder.readAndResetDelta();
  const float leftSpeed = speedFromTicks(leftDelta, dt);
  const float rightSpeed = speedFromTicks(rightDelta, dt);

  // Quando ha comando de movimento, aplica um feed-forward minimo para vencer atrito,
  // folga mecanica e zona morta comum em motores DC com L298N.
  int16_t leftPwm = 0;
  int16_t rightPwm = 0;
  if (fabsf(targetLeftMps) > 0.001f) {
    leftPwm = static_cast<int16_t>(_leftPid.compute(targetLeftMps, leftSpeed));
    leftPwm += targetLeftMps > 0.0f ? RobotConfig::MIN_MOVING_PWM : -RobotConfig::MIN_MOVING_PWM;
  } else {
    _leftPid.reset();
  }
  if (fabsf(targetRightMps) > 0.001f) {
    rightPwm = static_cast<int16_t>(_rightPid.compute(targetRightMps, rightSpeed));
    rightPwm += targetRightMps > 0.0f ? RobotConfig::MIN_MOVING_PWM : -RobotConfig::MIN_MOVING_PWM;
  } else {
    _rightPid.reset();
  }
  _leftMotor.setPWM(leftPwm);
  _rightMotor.setPWM(rightPwm);

  WheelSample out;
  out.leftSpeedMps = leftSpeed;
  out.rightSpeedMps = rightSpeed;
  out.leftTicks = _leftEncoder.getTicks();
  out.rightTicks = _rightEncoder.getTicks();
  return out;
}

void DriveBase::stop() {
  _leftPid.reset();
  _rightPid.reset();
  _leftMotor.brake();
  _rightMotor.brake();
}

float DriveBase::speedFromTicks(long ticks, float dt) const {
  if (dt <= 0.0f) return 0.0f;
  const float rev = static_cast<float>(ticks) / RobotConfig::TICKS_PER_REV;
  const float dist = rev * (2.0f * PI * RobotConfig::WHEEL_RADIUS_M);
  return dist / dt;
}
