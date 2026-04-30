#pragma once
#include <Arduino.h>
#include "../../src/SharedTypes.h"

class PurePursuit {
public:
  PurePursuit();
  void setLookahead(float meters);
  bool compute(const Pose2D& pose, const PathBuffer& path, float desiredSpeed, float& vLeft, float& vRight, bool& goalReached);

private:
  float _lookahead;
  uint16_t _lastIndex;
  bool findTarget(const Pose2D& pose, const PathBuffer& path, PathPoint& target);
};
