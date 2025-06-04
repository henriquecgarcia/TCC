#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "Arduino.h"    // For Serial (used in some inline/simple functions if DEBUG_PRINTS is on) and millis()
#include "esp_wifi.h"   // For WIFI_PS_NONE, WIFI_PS_MAX_MODEM, esp_wifi_set_ps()
#include <functional>   // For std::function

// Only include esp_pm.h if it's an ESP32 target, as it's specific
#ifdef CONFIG_IDF_TARGET_ESP32
    // No need to include esp_pm.h here if all its usage is in the .cpp
    // However, if any types from esp_pm.h were part of the public/protected interface
    // or used in inline methods, it would be needed.
#endif

// Define DEBUG_PRINTS globally in your project settings (e.g., platformio.ini build_flags)
// or uncomment here for local debugging
// #define DEBUG_PRINTS

enum power_mode_t {
    POWER_NORMAL,
    POWER_SAVING
};

class PowerManager {
private:
    power_mode_t power_mode;

    void configurePowerSettings(power_mode_t mode);

    unsigned long lastPowerCheck;
    // Static const integral members can be initialized directly in the header (C++11 and later)
    static const unsigned long powerCheckInterval = 1000; // 1 second check interval
    static const unsigned long idleTimeoutThreshold = 15000; // 15 seconds inactivity threshold

public:
    unsigned long *lastCommandReceived;     // Pointer to the timestamp of the last command
    std::function<bool()> isIdleCheck;      // Callback function to check for idle state

    // Prevent accidental copying
    PowerManager(const PowerManager&) = delete;
    PowerManager& operator=(const PowerManager&) = delete;

    PowerManager();
    ~PowerManager(); // Good practice to have a destructor

    bool setPowerMode(power_mode_t mode);
    power_mode_t getPowerMode() const;
    void printPowerMode() const;
    bool isPowerSaving() const;

    void loop();
};

#endif