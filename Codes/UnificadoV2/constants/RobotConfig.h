#pragma once

#include <Arduino.h>

// Configuracoes gerais do robo. Ajuste estes valores conforme a montagem real.
namespace RobotConfig {
  static constexpr const char* WIFI_SSID = "ESP32_ROBOT";
  static constexpr const char* WIFI_PASSWORD = "esp32robot";
  static constexpr const char* MDNS_NAME = "esp32-robo";

  static constexpr float WHEEL_DIAMETER_M = 0.065f;
  static constexpr float WHEEL_RADIUS_M = WHEEL_DIAMETER_M * 0.5f;
  static constexpr float WHEEL_BASE_M = 0.155f;
  static constexpr int ENCODER_TEETH = 10;
  static constexpr int ENCODER_EDGES_PER_TOOTH = 2;
  static constexpr float TICKS_PER_REV = ENCODER_TEETH * ENCODER_EDGES_PER_TOOTH;
  static constexpr float MAX_WHEEL_SPEED_MPS = 0.45f;
  static constexpr float MAX_LINEAR_SPEED_MPS = 0.28f;
  static constexpr float MAX_ANGULAR_SPEED_RADPS = 2.4f;
  static constexpr int16_t MIN_MOVING_PWM = 95;
  static constexpr float MANUAL_LINEAR_SPEED_MPS = 0.18f;
  static constexpr uint32_t MANUAL_LINEAR_DURATION_MS = 10000;
  static constexpr float MANUAL_TURN_SPEED_MPS = 0.14f;
  static constexpr uint32_t MANUAL_TURN_TIMEOUT_MS = 3200;
  static constexpr float MANUAL_TURN_TOLERANCE_RAD = 0.08f;

  static constexpr uint16_t CONTROL_PERIOD_MS = 20;
  static constexpr uint16_t SENSOR_PERIOD_MS = 25;
  static constexpr uint16_t NAV_PERIOD_MS = 100;
  static constexpr uint16_t TELEMETRY_PERIOD_MS = 500;
  static constexpr uint16_t MAP_TELEMETRY_PERIOD_MS = 30000;
  static constexpr uint16_t PATH_TELEMETRY_PERIOD_MS = 1500;
  static constexpr bool WIFI_ALLOW_SLEEP_IN_ECONOMY = false;

  static constexpr float CELL_SIZE_M = 0.10f;
  static constexpr uint16_t MAP_WIDTH = 32;
  static constexpr uint16_t MAP_HEIGHT = 32;
  static constexpr uint16_t MAP_CELLS = MAP_WIDTH * MAP_HEIGHT;
  static constexpr const char* MAP_FILE = "/map.bin";
  static constexpr uint16_t MAX_PATH_POINTS = 160;
  static constexpr float LOOKAHEAD_M = 0.28f;
  static constexpr float GOAL_TOLERANCE_M = 0.08f;

  static constexpr uint16_t OBSTACLE_MM = 260;
  static constexpr uint8_t OBSTACLE_PROJECTION_CELLS = 3;

  static constexpr uint32_t IDLE_TIMEOUT_MS = 30000;

  // L298N - lado esquerdo.
  static constexpr uint8_t LEFT_IN1 = 26;
  static constexpr uint8_t LEFT_IN2 = 27;
  static constexpr uint8_t LEFT_PWM = 25;

  // L298N - lado direito.
  static constexpr uint8_t RIGHT_IN1 = 14;
  static constexpr uint8_t RIGHT_IN2 = 12;
  static constexpr uint8_t RIGHT_PWM = 13;

  // Encoders. Use somente GPIOs seguros para interrupcao.
  static constexpr uint8_t LEFT_ENC_A = 34;
  static constexpr uint8_t LEFT_ENC_B = 35;
  static constexpr uint8_t RIGHT_ENC_A = 32;
  static constexpr uint8_t RIGHT_ENC_B = 33;

  // I2C compartilhado por MPU6050 e VL53L0X.
  static constexpr uint8_t I2C_SDA = 21;
  static constexpr uint8_t I2C_SCL = 22;

  // PWM ESP32.
  static constexpr uint8_t LEFT_PWM_CH = 0;
  static constexpr uint8_t RIGHT_PWM_CH = 1;
  static constexpr uint16_t PWM_FREQ = 20000;
  static constexpr uint8_t PWM_RES_BITS = 8;
}
