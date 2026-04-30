#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../../src/SharedTypes.h"

class ImuSensor {
public:
  bool begin();
  SensorSample read(uint16_t distanceMm, bool distanceValid, float dt);
  float getYawRad() const;

private:
  Adafruit_MPU6050 _mpu;
  float _gyroBiasZ;
  float _yawRad;
  bool calibrateGyroZ();
};
