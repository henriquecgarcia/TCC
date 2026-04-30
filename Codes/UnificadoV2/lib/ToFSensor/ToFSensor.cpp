#include "ToFSensor.h"

bool ToFSensor::begin() {
  if (!_sensor.begin()) {
    Serial.println("ERRO: VL53L0X nao encontrado.");
    return false;
  }
  return true;
}

bool ToFSensor::read(uint16_t& mm) {
  VL53L0X_RangingMeasurementData_t m;
  _sensor.rangingTest(&m, false);
  if (m.RangeStatus == 4 || m.RangeMilliMeter == 0) {
    mm = 8191;
    return false;
  }
  mm = static_cast<uint16_t>(m.RangeMilliMeter);
  return true;
}
