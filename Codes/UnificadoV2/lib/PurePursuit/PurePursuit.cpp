#include "PurePursuit.h"
#include "../../constants/RobotConfig.h"

PurePursuit::PurePursuit() : _lookahead(RobotConfig::LOOKAHEAD_M), _lastIndex(0) {}

void PurePursuit::setLookahead(float meters) {
  if (meters >= 0.05f && meters <= 2.0f) _lookahead = meters;
}

bool PurePursuit::compute(const Pose2D& pose, const PathBuffer& path, float desiredSpeed, float& vLeft, float& vRight, bool& goalReached) {
  vLeft = 0.0f;
  vRight = 0.0f;
  goalReached = false;
  if (path.count == 0) return false;

  const PathPoint& goal = path.points[path.count - 1];
  const float dxg = goal.x - pose.x;
  const float dyg = goal.y - pose.y;
  const float goalDist = sqrtf(dxg * dxg + dyg * dyg);
  if (goalDist <= RobotConfig::GOAL_TOLERANCE_M) {
    goalReached = true;
    _lastIndex = 0;
    return true;
  }

  PathPoint target;
  if (!findTarget(pose, path, target)) return false;

  const float dx = target.x - pose.x;
  const float dy = target.y - pose.y;
  const float localX = cosf(pose.theta) * dx + sinf(pose.theta) * dy;
  const float localY = -sinf(pose.theta) * dx + cosf(pose.theta) * dy;
  const float ld2 = localX * localX + localY * localY;
  if (ld2 < 0.0001f) return false;

  const float curvature = (2.0f * localY) / ld2;
  float v = desiredSpeed;
  if (fabsf(curvature) > 3.0f) v *= 0.55f;

  const float omega = curvature * v;
  vLeft = v - (omega * RobotConfig::WHEEL_BASE_M * 0.5f);
  vRight = v + (omega * RobotConfig::WHEEL_BASE_M * 0.5f);

  if (vLeft > RobotConfig::MAX_WHEEL_SPEED_MPS) vLeft = RobotConfig::MAX_WHEEL_SPEED_MPS;
  if (vLeft < -RobotConfig::MAX_WHEEL_SPEED_MPS) vLeft = -RobotConfig::MAX_WHEEL_SPEED_MPS;
  if (vRight > RobotConfig::MAX_WHEEL_SPEED_MPS) vRight = RobotConfig::MAX_WHEEL_SPEED_MPS;
  if (vRight < -RobotConfig::MAX_WHEEL_SPEED_MPS) vRight = -RobotConfig::MAX_WHEEL_SPEED_MPS;
  return true;
}

bool PurePursuit::findTarget(const Pose2D& pose, const PathBuffer& path, PathPoint& target) {
  if (_lastIndex >= path.count) _lastIndex = 0;

  uint16_t best = _lastIndex;
  float bestDist = 100000.0f;
  for (uint16_t i = _lastIndex; i < path.count; i++) {
    const float dx = path.points[i].x - pose.x;
    const float dy = path.points[i].y - pose.y;
    const float d = sqrtf(dx * dx + dy * dy);
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  _lastIndex = best;

  for (uint16_t i = best; i < path.count; i++) {
    const float dx = path.points[i].x - pose.x;
    const float dy = path.points[i].y - pose.y;
    const float d = sqrtf(dx * dx + dy * dy);
    if (d >= _lookahead) {
      target = path.points[i];
      _lastIndex = i;
      return true;
    }
  }

  target = path.points[path.count - 1];
  return true;
}
