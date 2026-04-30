#pragma once
#include <Arduino.h>
#include "../GridMap/GridMap.h"
#include "../../src/SharedTypes.h"

class AStarPlanner {
public:
  explicit AStarPlanner(GridMap& map);
  bool plan(uint16_t sx, uint16_t sy, uint16_t gx, uint16_t gy, PathBuffer& outPath);

private:
  struct Node {
    int16_t parent;
    uint16_t g;
    uint16_t f;
    int8_t dir;
    bool open;
    bool closed;
  };

  GridMap& _map;
  Node _nodes[RobotConfig::MAP_CELLS];
  static uint16_t idx(uint16_t x, uint16_t y);
  static uint16_t heuristic(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by);
  int16_t popBestOpen();
  bool buildPath(uint16_t startIdx, uint16_t goalIdx, PathBuffer& outPath);
};
