#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB) : _pinA(pinA), _pinB(pinB), _ticks(0), _delta(0) {}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  attachInterruptArg(_pinA, isrA, this, CHANGE);
  attachInterruptArg(_pinB, isrB, this, CHANGE);
}

long Encoder::getTicks() const {
  noInterrupts();
  const long v = _ticks;
  interrupts();
  return v;
}

long Encoder::readAndResetDelta() {
  noInterrupts();
  const long v = _delta;
  _delta = 0;
  interrupts();
  return v;
}

void Encoder::reset() {
  noInterrupts();
  _ticks = 0;
  _delta = 0;
  interrupts();
}

void IRAM_ATTR Encoder::handleA() {
  const int a = digitalRead(_pinA);
  const int b = digitalRead(_pinB);
  const int dir = (a == b) ? 1 : -1;
  _ticks += dir;
  _delta += dir;
}

void IRAM_ATTR Encoder::handleB() {
  const int a = digitalRead(_pinA);
  const int b = digitalRead(_pinB);
  const int dir = (a != b) ? 1 : -1;
  _ticks += dir;
  _delta += dir;
}

void IRAM_ATTR Encoder::isrA(void* arg) { static_cast<Encoder*>(arg)->handleA(); }
void IRAM_ATTR Encoder::isrB(void* arg) { static_cast<Encoder*>(arg)->handleB(); }
