#pragma once

#include <Arduino.h>
#include "../constants/RobotConfig.h"

struct Pose2D {
  float x;
  float y;
  float theta;
};

struct WheelSample {
  float leftSpeedMps;
  float rightSpeedMps;
  long leftTicks;
  long rightTicks;
};

struct SensorSample {
  float gyroZRadps;
  float yawRad;
  uint16_t distanceMm;
  bool distanceValid;
};

struct PathPoint {
  float x;
  float y;
  uint16_t gx;
  uint16_t gy;
};

struct PathBuffer {
  PathPoint points[RobotConfig::MAX_PATH_POINTS];
  uint16_t count;
};

struct NavGoal {
  uint16_t gx;
  uint16_t gy;
  bool active;
};

static inline float normalizeAngle(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static inline bool worldToGrid(float x, float y, uint16_t& gx, uint16_t& gy) {
  if (x < 0.0f || y < 0.0f) return false;
  const uint16_t ix = static_cast<uint16_t>(x / RobotConfig::CELL_SIZE_M);
  const uint16_t iy = static_cast<uint16_t>(y / RobotConfig::CELL_SIZE_M);
  if (ix >= RobotConfig::MAP_WIDTH || iy >= RobotConfig::MAP_HEIGHT) return false;
  gx = ix;
  gy = iy;
  return true;
}

static inline void gridToWorld(uint16_t gx, uint16_t gy, float& x, float& y) {
  x = (static_cast<float>(gx) + 0.5f) * RobotConfig::CELL_SIZE_M;
  y = (static_cast<float>(gy) + 0.5f) * RobotConfig::CELL_SIZE_M;
}
