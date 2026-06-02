#pragma once

#include <Arduino.h>
#include "MediaMovel.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"

class Gyroscope {
private:
	Adafruit_MPU6050 sensor;  // sem new/delete
	unsigned long lastRead = 0;
	static const unsigned long readInterval = 20; // ms entre leituras

	MediaMovel gyroX{1};
	MediaMovel gyroY{1};
	MediaMovel gyroZ{1};

	// previne cópia acidental (dois objetos lutando pelo mesmo hardware)
	Gyroscope(const Gyroscope&) = delete;
	Gyroscope& operator=(const Gyroscope&) = delete;

	bool firstRead = true; // para evitar leituras iniciais erradas
	// offsets calibrados para evitar drift
	float offsetX = 0.06; // ajuste fino do giroscópio X
	float offsetY = -0.03; // ajuste fino do giroscópio Y
	float offsetZ = -0.03; // ajuste fino do giroscópio Z

	MediaMovel accelX{1};
	MediaMovel accelY{1};
	MediaMovel accelZ{1};
	float cachedTempC = 0.0f;

	float offsetAccelX = 0.0; // ajuste fino do acelerômetro X
	float offsetAccelY = 0.0; // ajuste fino do acelerômetro Y
	float offsetAccelZ = 0.0; // ajuste fino do acelerômetro Z

	// unsigned long lastWSUpdate = 0;
public:
	Gyroscope() = default;  // construtor padrão

	void setup();

	float getAccelerometerX() const {
		return accelX.get();
	}
	float getAccelerometerY() const {
		return accelY.get();
	}
	float getAccelerometerZ() const {
		return accelZ.get();
	}

	float getGyroscopeX() const {
		return gyroX.get();
	}
	float getGyroscopeY() const {
		return gyroY.get();
	}
	float getGyroscopeZ() const {
		return gyroZ.get();
	}
	float getTemperatureC() const {
		return cachedTempC;
	}

	void loop();
};