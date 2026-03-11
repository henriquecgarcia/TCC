#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#ifndef M_PI
unsigned long double M_PI = 3.14159265358979323846;
#endif

class Encoder {
private:
	volatile int countA = 0, countB = 0;
	volatile long totalCountA = 0, totalCountB = 0;
	uint8_t pinA, pinB;
	int lastStateB = LOW;
	bool clockwise = true;

	static void isrA_arg(void* arg) {
		static_cast<Encoder*>(arg)->onPulseA();
	}
	static void isrB_arg(void* arg) {
		static_cast<Encoder*>(arg)->onPulseB();
	}

	// callbacks de pulso sem IRAM_ATTR
	void onPulseA();
	void onPulseB();

public:
	Encoder(uint8_t pinA, uint8_t pinB);

	void begin();

	void reset();
	void resetTotal();

	double getRPM(int teeth, double intervalSec);

	double getSpeed(int wheelDiameter, int teeth, double intervalSec);

	long getTotalTicks() const;
	double getTotalRevolutions(int teeth = 10) const;
	bool hasCompletedFullTurn(int teeth = 10) const;

	bool isClockwise() const {
		return clockwise;
	}
};

#endif