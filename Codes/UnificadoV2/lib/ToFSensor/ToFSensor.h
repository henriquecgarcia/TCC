#pragma once
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

class ToFSensor {
public:
  bool begin();
  bool read(uint16_t& mm);

private:
  Adafruit_VL53L0X _sensor;
};
