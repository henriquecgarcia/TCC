#pragma once
#include <Arduino.h>

class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB);
  void begin();
  long getTicks() const;
  long readAndResetDelta();
  void reset();

private:
  uint8_t _pinA;
  uint8_t _pinB;
  volatile long _ticks;
  volatile long _delta;

  void IRAM_ATTR handleA();
  void IRAM_ATTR handleB();
  static void IRAM_ATTR isrA(void* arg);
  static void IRAM_ATTR isrB(void* arg);
};
