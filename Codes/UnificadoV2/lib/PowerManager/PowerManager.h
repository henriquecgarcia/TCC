#pragma once
#include <Arduino.h>
#include <esp_wifi.h>
#include "../../constants/RobotConfig.h"

class PowerManager {
public:
  enum Mode : uint8_t { NORMAL, ECONOMY };
  PowerManager();
  void begin(uint32_t* lastCommandMs, bool* robotActive);
  void loop();
  void forceNormal();
  Mode mode() const;

private:
  uint32_t* _lastCommandMs;
  bool* _robotActive;
  uint32_t _lastCheckMs;
  Mode _mode;
  void setMode(Mode mode);
};
