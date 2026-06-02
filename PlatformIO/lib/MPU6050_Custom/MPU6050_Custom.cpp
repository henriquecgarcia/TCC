#pragma once

#include <Arduino.h>
#include "MediaMovel.h"
#include "MPU6050_Custom.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"

void Gyroscope::setup() {
	if (!sensor.begin()) {
		delay(1000);
		ESP.restart();
		return;
	}
	sensor.setAccelerometerRange(MPU6050_RANGE_2_G);
	sensor.setGyroRange(MPU6050_RANGE_500_DEG);
	sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void Gyroscope::loop() {
	unsigned long now = millis();
	if (now - lastRead < readInterval) {
		return;  // evita leituras muito frequentes
	}
	lastRead = now;

	// faz só uma chamada ao sensor por ciclo
	sensors_event_t a, g, temp;
	sensor.getEvent(&a, &g, &temp);
	cachedTempC = temp.temperature;

	if (firstRead) {
		// calibra offsets na primeira leitura
		offsetX = g.gyro.x;
		offsetY = g.gyro.y;
		offsetZ = g.gyro.z;
		firstRead = false;

		offsetAccelX = a.acceleration.x;
		offsetAccelY = a.acceleration.y;
		offsetAccelZ = a.acceleration.z;
	}

	// Atualiza as médias móveis com offsets calibrados
	gyroX.add(g.gyro.x - offsetX);
	gyroY.add(g.gyro.y - offsetY);
	gyroZ.add(g.gyro.z - offsetZ);
	accelX.add(a.acceleration.x - offsetAccelX);
	accelY.add(a.acceleration.y - offsetAccelY);
	accelZ.add(a.acceleration.z - offsetAccelZ);
}