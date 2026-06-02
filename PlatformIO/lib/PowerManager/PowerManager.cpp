#include "PowerManager.h"

// ESP-IDF specific power management includes, only if compiling for ESP32
#ifdef CONFIG_IDF_TARGET_ESP32
    #include "esp_pm.h"
#endif

#ifndef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
    #define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 240
#endif

PowerManager::PowerManager() : power_mode(POWER_NORMAL), lastPowerCheck(0), lastCommandReceived(nullptr), isIdleCheck(nullptr) {
    // Apply NORMAL mode settings immediately
    configurePowerSettings(POWER_NORMAL);
    // Ensure Wi-Fi starts in WIFI_PS_NONE (no power save)
    // This should ideally be called after esp_wifi_start() has been called elsewhere in your setup
    // For now, we'll call it, but be mindful of the Wi-Fi initialization sequence.
    esp_wifi_set_ps(WIFI_PS_NONE);
    #ifdef DEBUG_PRINTS
        Serial.println("PowerManager initialized in NORMAL mode.");
    #endif
}

PowerManager::~PowerManager() {
    // Cleanup if necessary
}

void PowerManager::configurePowerSettings(power_mode_t mode) {
    esp_err_t err = ESP_FAIL; // Initialize to a failing state

#ifdef CONFIG_IDF_TARGET_ESP32
    if (mode == POWER_SAVING) {
        esp_pm_config_esp32_t pmConfig = {
            .max_freq_mhz       = 160, // Example: Reduce max CPU frequency
            .min_freq_mhz       = 80,  // Example: Can be XTAL frequency or APB frequency
            .light_sleep_enable = true
        };
        err = esp_pm_configure(&pmConfig);
        #ifdef DEBUG_PRINTS
            Serial.printf("esp_pm_configure (ECONOMIA): %s\n", esp_err_to_name(err));
        #endif
    } else { // POWER_NORMAL
        esp_pm_config_esp32_t pmConfig = {
            .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, // Use default max frequency
            .min_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, // Can be set same as max, or lower if DFS is complexly managed
            .light_sleep_enable = false
        };
        err = esp_pm_configure(&pmConfig);
        #ifdef DEBUG_PRINTS
            Serial.printf("esp_pm_configure (NORMAL): %s\n", esp_err_to_name(err));
        #endif
    }

    if (err != ESP_OK) {
        #ifdef DEBUG_PRINTS
            Serial.println("⚠️ Falha ao configurar gerenciamento de energia ESP-IDF!");
        #endif
    }
#else
    #ifdef DEBUG_PRINTS
        Serial.println("⚠️ Gerenciamento de energia ESP-IDF (esp_pm) não suportado ou desabilitado nesta plataforma/configuração.");
    #endif
    // To avoid unused variable warning if DEBUG_PRINTS is off and not ESP32
    (void)err; 
#endif // CONFIG_IDF_TARGET_ESP32
    (void)mode; // To avoid unused variable warning if not ESP32
}

bool PowerManager::setPowerMode(power_mode_t mode) {
    if (mode == this->power_mode) {
        #ifdef DEBUG_PRINTS
            Serial.println("Já está no modo solicitado.");
        #endif
        return false; // No change made
    }

    configurePowerSettings(mode);

    if (mode == POWER_SAVING) {
        // Wi-Fi power save mode. This typically needs esp_wifi_start() to have been called.
        #ifdef DEBUG_PRINTS
            Serial.println("Colocando WiFi em modo de economia (MAX_MODEM)...");
        #endif
        esp_err_t wifi_ps_err = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        #ifdef DEBUG_PRINTS
            if (wifi_ps_err == ESP_OK) {
                Serial.println("WiFi em modo de economia!");
            } else {
                Serial.printf("⚠️ Falha ao colocar WiFi em modo de economia: %s\n", esp_err_to_name(wifi_ps_err));
            }
            Serial.println("Modo de energia alterado para ECONOMIA");
        #endif
    } else { // POWER_NORMAL
        #ifdef DEBUG_PRINTS
            Serial.println("Colocando WiFi em modo normal (NONE)...");
        #endif
        esp_err_t wifi_ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
        #ifdef DEBUG_PRINTS
             if (wifi_ps_err == ESP_OK) {
                Serial.println("WiFi em modo normal!");
            } else {
                Serial.printf("⚠️ Falha ao colocar WiFi em modo normal: %s\n", esp_err_to_name(wifi_ps_err));
            }
            Serial.println("Modo de energia alterado para NORMAL");
        #endif
    }

    this->power_mode = mode;
    return true; // Mode changed
}

power_mode_t PowerManager::getPowerMode() const {
    return this->power_mode;
}

void PowerManager::printPowerMode() const {
    #ifdef DEBUG_PRINTS
        if (this->power_mode == POWER_NORMAL) {
            Serial.println("Modo de energia: NORMAL");
        } else {
            Serial.println("Modo de energia: ECONOMIA");
        }
    #endif
}

bool PowerManager::isPowerSaving() const {
    return this->power_mode == POWER_SAVING;
}

void PowerManager::loop() {
    if (!lastCommandReceived || isIdleCheck == nullptr) {
        #ifdef DEBUG_PRINTS
            // This check might be too frequent if loop() is called very fast.
            // Consider a flag or a less frequent print.
            // static unsigned long lastWarningTime = 0;
            // if (millis() - lastWarningTime > 5000) { // Print warning every 5s
            //    Serial.println("⚠️ PowerManager: lastCommandReceived ou isIdleCheck não configurados!");
            //    lastWarningTime = millis();
            // }
        #endif
        return;
    }

    unsigned long now = millis();
    if (now - lastPowerCheck < powerCheckInterval) {
        return;  // Avoid checking too frequently
    }
    lastPowerCheck = now;

    // Check if the system is idle based on the provided timestamp and callback
    bool isCurrentlyIdle = (now - *lastCommandReceived > idleTimeoutThreshold) && isIdleCheck();

    if (isCurrentlyIdle && this->power_mode != POWER_SAVING) {
        #ifdef DEBUG_PRINTS
            Serial.println("🟡 PowerManager: Entrando em modo de ECONOMIA por inatividade...");
        #endif
        setPowerMode(POWER_SAVING);
    } else if (!isCurrentlyIdle && this->power_mode != POWER_NORMAL) {
        #ifdef DEBUG_PRINTS
            Serial.println("🔵 PowerManager: Atividade detectada! Voltando ao modo NORMAL...");
        #endif
        setPowerMode(POWER_NORMAL);
    }
}