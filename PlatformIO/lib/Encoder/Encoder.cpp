#include "Encoder.h"

#ifndef M_PI
unsigned long double M_PI = 3.14159265358979323846;
#endif

void Encoder::onPulseA() {
	countA++;
}
void Encoder::onPulseB() {
	countB++;
	int current = digitalRead(pinB);
	if (lastStateB == LOW && current == HIGH) {
		clockwise = (digitalRead(pinA) == LOW);
	}
	lastStateB = current;
}

Encoder::Encoder(uint8_t pinA, uint8_t pinB) : pinA(pinA), pinB(pinB) {
	countA = countB = 0;
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

double Encoder::getRPM(int teeth = 10, double intervalSec = 0.1) {
	int pulses = (countA + countB) / 2;
	double revs = pulses / double(teeth * 2);
	return (revs / intervalSec) * 60.0;
}

double Encoder::getSpeed(int wheelDiameter = 65, int teeth = 10, double intervalSec = 0.1) {
	double rpm = getRPM(teeth, intervalSec);
	return (rpm * wheelDiameter * M_PI) / 1000.0; // mm/s
}