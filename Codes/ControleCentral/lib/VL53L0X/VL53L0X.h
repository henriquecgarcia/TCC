#ifndef VL53L0X_H
#define VL53L0X_H

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

class VL53L0X {
private:
	Adafruit_VL53L0X sensor; // sem alocação dinâmica
	unsigned long lastRead = 0; // inicialização inline
	int last_reading = 0;
	static const unsigned long readInterval = 100; // ms entre leituras

	// previne cópia acidental (dois objetos no mesmo hardware)
	VL53L0X(const VL53L0X&) = delete;
	VL53L0X& operator=(const VL53L0X&) = delete;

public:
	VL53L0X();

	void setup();

	int loop();

	int getLastReading() const {
		return last_reading;
	}

	void reset();
	void printLastReading() const;
	void printSensorInfo() const;
};

#endif