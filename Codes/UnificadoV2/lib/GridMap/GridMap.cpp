#include "GridMap.h"

bool GridMap::begin(const char* path) {
  _path = path;
  clearDynamic();
  File f = SPIFFS.open(_path, "r");
  if (!f) {
    Serial.println("ERRO: map.bin nao encontrado no SPIFFS.");
    return false;
  }
  const size_t expected = RobotConfig::MAP_CELLS;
  const bool ok = f.size() >= expected;
  f.close();
  if (!ok) Serial.println("ERRO: map.bin menor que o esperado.");
  return ok;
}

bool GridMap::isOccupied(uint16_t x, uint16_t y) {
  if (!valid(x, y)) return true;
  if (_dynamic[index(x, y)] == 1) return true;
  uint8_t v = 1;
  if (!readStatic(x, y, v)) return true;
  return v != 0;
}

bool GridMap::setDynamic(uint16_t x, uint16_t y, bool occupied) {
  if (!valid(x, y)) return false;
  _dynamic[index(x, y)] = occupied ? 1 : 0;
  return true;
}

void GridMap::clearDynamic() {
  memset(_dynamic, 0, sizeof(_dynamic));
}

uint8_t GridMap::cellForTelemetry(uint16_t x, uint16_t y) {
  if (!valid(x, y)) return 1;
  if (_dynamic[index(x, y)] == 1) return 2;
  uint8_t v = 1;
  if (!readStatic(x, y, v)) return 1;
  return v ? 1 : 0;
}

bool GridMap::readStatic(uint16_t x, uint16_t y, uint8_t& value) {
  if (!valid(x, y)) return false;
  File f = SPIFFS.open(_path, "r");
  if (!f) return false;
  const uint32_t pos = index(x, y);
  if (!f.seek(pos, SeekSet)) {
    f.close();
    return false;
  }
  const int c = f.read();
  f.close();
  if (c < 0) return false;
  value = static_cast<uint8_t>(c);
  return true;
}

bool GridMap::valid(uint16_t x, uint16_t y) const {
  return x < RobotConfig::MAP_WIDTH && y < RobotConfig::MAP_HEIGHT;
}

uint32_t GridMap::index(uint16_t x, uint16_t y) const {
  return static_cast<uint32_t>(y) * RobotConfig::MAP_WIDTH + x;
}
