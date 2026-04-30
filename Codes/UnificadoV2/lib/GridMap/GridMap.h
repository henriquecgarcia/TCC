#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include "../../constants/RobotConfig.h"

class GridMap {
public:
  bool begin(const char* path);
  bool isOccupied(uint16_t x, uint16_t y);
  bool setDynamic(uint16_t x, uint16_t y, bool occupied);
  void clearDynamic();
  uint8_t cellForTelemetry(uint16_t x, uint16_t y);

private:
  const char* _path;
  uint8_t _dynamic[RobotConfig::MAP_CELLS];
  bool readStatic(uint16_t x, uint16_t y, uint8_t& value);
  bool valid(uint16_t x, uint16_t y) const;
  uint32_t index(uint16_t x, uint16_t y) const;
};
