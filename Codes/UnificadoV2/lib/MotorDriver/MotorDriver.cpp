#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel)
  : _in1(in1), _in2(in2), _pwmPin(pwmPin), _pwmChannel(pwmChannel), _maxPwm(255) {}

void MotorDriver::begin(uint16_t pwmFreq, uint8_t pwmResolutionBits) {
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  ledcSetup(_pwmChannel, pwmFreq, pwmResolutionBits);
  ledcAttachPin(_pwmPin, _pwmChannel);
  _maxPwm = static_cast<uint8_t>((1 << pwmResolutionBits) - 1);
  coast();
}

void MotorDriver::setPWM(int16_t pwm) {
  if (pwm > _maxPwm) pwm = _maxPwm;
  if (pwm < -static_cast<int16_t>(_maxPwm)) pwm = -static_cast<int16_t>(_maxPwm);

  if (pwm > 0) {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    ledcWrite(_pwmChannel, pwm);
  } else if (pwm < 0) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    ledcWrite(_pwmChannel, -pwm);
  } else {
    coast();
  }
}

void MotorDriver::brake() {
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  ledcWrite(_pwmChannel, 0);
}

void MotorDriver::coast() {
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
  ledcWrite(_pwmChannel, 0);
}
