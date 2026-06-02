#include "Encoder.h"

void Encoder::onPulseA() {
	countA++;
	totalCountA++;
}
void Encoder::onPulseB() {
	countB++;
	totalCountB++;
	int current = digitalRead(pinB);
	if (lastStateB == LOW && current == HIGH) {
		clockwise = (digitalRead(pinA) == LOW);
	}
	lastStateB = current;
}

Encoder::Encoder(uint8_t pinA, uint8_t pinB) : pinA(pinA), pinB(pinB) {
	countA = countB = 0;
	totalCountA = totalCountB = 0;
	lastStateB = LOW;
	clockwise = true;
}

void Encoder::begin() {
	pinMode(pinA, INPUT_PULLUP);
	pinMode(pinB, INPUT_PULLUP);

	attachInterruptArg(pinA, isrA_arg, this, CHANGE);
	attachInterruptArg(pinB, isrB_arg, this, CHANGE);
}

void Encoder::reset() {
	countA = countB = 0;
}

void Encoder::resetTotal() {
	noInterrupts();
	totalCountA = totalCountB = 0;
	interrupts();
}

double Encoder::getRPM(int teeth = 10, double intervalSec = 0.1) {
	int pulses = (countA + countB) / 2;
	double revs = pulses / double(teeth * 2);
	return (revs / intervalSec) * 60.0;
}

double Encoder::getSpeed(int wheelDiameter = 65, int teeth = 10, double intervalSec = 0.1) {
	double rpm = getRPM(teeth, intervalSec);
	return (rpm * wheelDiameter * M_PI) / 1000.0; // mm/s
}

long Encoder::getTotalTicks() const {
	noInterrupts();
	const long ticks = (totalCountA + totalCountB) / 2;
	interrupts();
	return ticks;
}

double Encoder::getTotalRevolutions(int teeth) const {
	if (teeth <= 0) {
		return 0.0;
	}
	const long ticks = getTotalTicks();
	return ticks / double(teeth * 2);
}

bool Encoder::hasCompletedFullTurn(int teeth) const {
	return fabs(getTotalRevolutions(teeth)) >= 1.0;
}