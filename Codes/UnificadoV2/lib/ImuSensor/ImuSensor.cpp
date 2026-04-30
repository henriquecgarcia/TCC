#include "ImuSensor.h"

bool ImuSensor::begin() {
  if (!_mpu.begin()) {
    Serial.println("ERRO: MPU6050 nao encontrado.");
    return false;
  }
  _mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  _yawRad = 0.0f;
  return calibrateGyroZ();
}

SensorSample ImuSensor::read(uint16_t distanceMm, bool distanceValid, float dt) {
  sensors_event_t a, g, temp;
  _mpu.getEvent(&a, &g, &temp);
  const float gz = g.gyro.z - _gyroBiasZ;
  if (dt > 0.0f && dt < 0.5f) {
    _yawRad = normalizeAngle(_yawRad + gz * dt);
  }

  SensorSample s;
  s.gyroZRadps = gz;
  s.yawRad = _yawRad;
  s.distanceMm = distanceMm;
  s.distanceValid = distanceValid;
  return s;
}

float ImuSensor::getYawRad() const {
  return _yawRad;
}

bool ImuSensor::calibrateGyroZ() {
  sensors_event_t a, g, temp;
  float sum = 0.0f;
  constexpr uint16_t samples = 200;
  for (uint16_t i = 0; i < samples; i++) {
    _mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(3);
  }
  _gyroBiasZ = sum / samples;
  return true;
}
