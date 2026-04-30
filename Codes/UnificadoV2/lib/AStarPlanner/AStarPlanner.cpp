#include "AStarPlanner.h"

AStarPlanner::AStarPlanner(GridMap& map) : _map(map) {}

bool AStarPlanner::plan(uint16_t sx, uint16_t sy, uint16_t gx, uint16_t gy, PathBuffer& outPath) {
  outPath.count = 0;
  if (sx >= RobotConfig::MAP_WIDTH || sy >= RobotConfig::MAP_HEIGHT) return false;
  if (gx >= RobotConfig::MAP_WIDTH || gy >= RobotConfig::MAP_HEIGHT) return false;
  if (_map.isOccupied(sx, sy) || _map.isOccupied(gx, gy)) return false;

  for (uint16_t i = 0; i < RobotConfig::MAP_CELLS; i++) {
    _nodes[i].parent = -1;
    _nodes[i].g = 65535;
    _nodes[i].f = 65535;
    _nodes[i].dir = -1;
    _nodes[i].open = false;
    _nodes[i].closed = false;
  }

  const uint16_t start = idx(sx, sy);
  const uint16_t goal = idx(gx, gy);
  _nodes[start].g = 0;
  _nodes[start].f = heuristic(sx, sy, gx, gy);
  _nodes[start].open = true;

  static const int8_t dx[4] = {1, -1, 0, 0};
  static const int8_t dy[4] = {0, 0, 1, -1};

  while (true) {
    const int16_t current = popBestOpen();
    if (current < 0) return false;
    if (current == goal) return buildPath(start, goal, outPath);

    _nodes[current].open = false;
    _nodes[current].closed = true;
    const uint16_t cx = current % RobotConfig::MAP_WIDTH;
    const uint16_t cy = current / RobotConfig::MAP_WIDTH;

    for (uint8_t d = 0; d < 4; d++) {
      const int nx = static_cast<int>(cx) + dx[d];
      const int ny = static_cast<int>(cy) + dy[d];
      if (nx < 0 || ny < 0 || nx >= RobotConfig::MAP_WIDTH || ny >= RobotConfig::MAP_HEIGHT) continue;
      if (_map.isOccupied(nx, ny)) continue;
      const uint16_t ni = idx(nx, ny);
      if (_nodes[ni].closed) continue;

      const uint16_t turnPenalty = (_nodes[current].dir >= 0 && _nodes[current].dir != static_cast<int8_t>(d)) ? 3 : 0;
      const uint16_t tentative = _nodes[current].g + 10 + turnPenalty;
      if (!_nodes[ni].open || tentative < _nodes[ni].g) {
        _nodes[ni].parent = current;
        _nodes[ni].g = tentative;
        _nodes[ni].f = tentative + heuristic(nx, ny, gx, gy);
        _nodes[ni].dir = d;
        _nodes[ni].open = true;
      }
    }
  }
}

uint16_t AStarPlanner::idx(uint16_t x, uint16_t y) {
  return y * RobotConfig::MAP_WIDTH + x;
}

uint16_t AStarPlanner::heuristic(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by) {
  const uint16_t dx = ax > bx ? ax - bx : bx - ax;
  const uint16_t dy = ay > by ? ay - by : by - ay;
  return static_cast<uint16_t>((dx + dy) * 10);
}

int16_t AStarPlanner::popBestOpen() {
  uint16_t bestF = 65535;
  int16_t best = -1;
  for (uint16_t i = 0; i < RobotConfig::MAP_CELLS; i++) {
    if (_nodes[i].open && _nodes[i].f < bestF) {
      bestF = _nodes[i].f;
      best = i;
    }
  }
  return best;
}

bool AStarPlanner::buildPath(uint16_t startIdx, uint16_t goalIdx, PathBuffer& outPath) {
  uint16_t reversed[RobotConfig::MAX_PATH_POINTS];
  uint16_t count = 0;
  int16_t cur = goalIdx;
  while (cur >= 0 && count < RobotConfig::MAX_PATH_POINTS) {
    reversed[count++] = static_cast<uint16_t>(cur);
    if (cur == startIdx) break;
    cur = _nodes[cur].parent;
  }
  if (count == 0 || reversed[count - 1] != startIdx) return false;

  outPath.count = count;
  for (uint16_t i = 0; i < count; i++) {
    const uint16_t id = reversed[count - 1 - i];
    const uint16_t gx = id % RobotConfig::MAP_WIDTH;
    const uint16_t gy = id / RobotConfig::MAP_WIDTH;
    outPath.points[i].gx = gx;
    outPath.points[i].gy = gy;
    gridToWorld(gx, gy, outPath.points[i].x, outPath.points[i].y);
  }
  return true;
}
