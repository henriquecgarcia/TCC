#include "PowerManager.h"
#include <esp_pm.h>

PowerManager::PowerManager() : _lastCommandMs(nullptr), _robotActive(nullptr), _lastCheckMs(0), _mode(ECONOMY) {}

void PowerManager::begin(uint32_t* lastCommandMs, bool* robotActive) {
  _lastCommandMs = lastCommandMs;
  _robotActive = robotActive;
  setMode(NORMAL);
}

void PowerManager::loop() {
  if (_lastCommandMs == nullptr || _robotActive == nullptr) return;
  const uint32_t now = millis();
  if (now - _lastCheckMs < 1000) return;
  _lastCheckMs = now;

  const bool idle = !(*_robotActive) && (now - *_lastCommandMs > RobotConfig::IDLE_TIMEOUT_MS);
  if (idle && _mode != ECONOMY) setMode(ECONOMY);
  if (!idle && _mode != NORMAL) setMode(NORMAL);
}

void PowerManager::forceNormal() {
  setMode(NORMAL);
}

PowerManager::Mode PowerManager::mode() const {
  return _mode;
}

void PowerManager::setMode(Mode mode) {
  if (mode == _mode) return;
  _mode = mode;

  if (mode == ECONOMY) {
    // Economia sem colocar o Wi-Fi em modem sleep por padrao.
    // Em modo Access Point, WIFI_PS_MAX_MODEM pode causar quedas ou latencia alta no WebSocket.
    if (RobotConfig::WIFI_ALLOW_SLEEP_IN_ECONOMY) {
      esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    } else {
      esp_wifi_set_ps(WIFI_PS_NONE);
    }
    setCpuFrequencyMhz(160);
  } else {
    setCpuFrequencyMhz(240);
    esp_wifi_set_ps(WIFI_PS_NONE);
  }
}
