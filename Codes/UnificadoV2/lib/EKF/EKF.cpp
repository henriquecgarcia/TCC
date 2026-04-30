#include "EKF.h"
#include "../../constants/RobotConfig.h"

EKF::EKF() : _q{0.0008f, 0.0008f, 0.0015f}, _rTheta(0.025f) {
  reset(0.05f, 0.05f, 0.0f);
}

void EKF::reset(float x, float y, float theta) {
  _x.x = x;
  _x.y = y;
  _x.theta = normalizeAngle(theta);
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) _p[r][c] = (r == c) ? 0.02f : 0.0f;
  }
}

void EKF::predict(float vLeft, float vRight, float dt) {
  if (dt <= 0.0f || dt > 0.2f) return;
  const float v = 0.5f * (vRight + vLeft);
  const float w = (vRight - vLeft) / RobotConfig::WHEEL_BASE_M;
  const float th = _x.theta;

  _x.x += v * cosf(th) * dt;
  _x.y += v * sinf(th) * dt;
  _x.theta = normalizeAngle(_x.theta + w * dt);

  const float f02 = -v * sinf(th) * dt;
  const float f12 =  v * cosf(th) * dt;
  float np[3][3];

  // P = FPF' + Q, com F triangular simples.
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) np[r][c] = _p[r][c];
  }
  np[0][0] += f02 * _p[2][0] + _p[0][2] * f02 + f02 * _p[2][2] * f02 + _q[0];
  np[0][1] += f02 * _p[2][1] + _p[0][2] * f12 + f02 * _p[2][2] * f12;
  np[1][0] += f12 * _p[2][0] + _p[1][2] * f02 + f12 * _p[2][2] * f02;
  np[1][1] += f12 * _p[2][1] + _p[1][2] * f12 + f12 * _p[2][2] * f12 + _q[1];
  np[2][2] += _q[2];

  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) _p[r][c] = np[r][c];
  }
}

void EKF::updateTheta(float measuredTheta) {
  const float y = normalizeAngle(measuredTheta - _x.theta);
  const float s = _p[2][2] + _rTheta;
  if (s <= 0.0f) return;
  const float k0 = _p[0][2] / s;
  const float k1 = _p[1][2] / s;
  const float k2 = _p[2][2] / s;

  _x.x += k0 * y;
  _x.y += k1 * y;
  _x.theta = normalizeAngle(_x.theta + k2 * y);

  for (uint8_t c = 0; c < 3; c++) {
    _p[0][c] -= k0 * _p[2][c];
    _p[1][c] -= k1 * _p[2][c];
    _p[2][c] -= k2 * _p[2][c];
  }
}

Pose2D EKF::pose() const {
  return _x;
}
