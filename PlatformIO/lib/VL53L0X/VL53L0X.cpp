#include "VL53L0X.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

void VL53L0X::setup() {
	if (!sensor.begin()) {
		Serial.println("Falha ao encontrar o sensor VL53L0X");
		Serial.println("Reiniciando ESP32...");
		delay(1000); // espera 1 segundo antes de reiniciar
		ESP.restart(); // reinicia o ESP32 se o sensor não for encontrado
		return;
	}
	Serial.println("Sensor VL53L0X encontrado!");
}

VL53L0X::VL53L0X() : sensor(), lastRead(0), last_reading(0) {
	sensor.startMeasurement();
}

int VL53L0X::loop() {
	unsigned long now = millis();
	if (now - lastRead < readInterval) {
		return last_reading;
	}
	lastRead = now;

	VL53L0X_RangingMeasurementData_t measure;
	sensor.rangingTest(&measure, false);

	Serial.print("Distancia: ");
	if (measure.RangeStatus != 4) { // se não estiver fora de alcance
		last_reading = measure.RangeMilliMeter;

		Serial.print(last_reading);
		Serial.println(" mm");
	} else {
		Serial.println("Fora do alcance");
		last_reading = 99999;
	}
	return last_reading;
}

void VL53L0X::reset() {
	lastRead = 0;
	last_reading = 0;
	sensor.stopMeasurement();
	sensor.startMeasurement();
	Serial.println("Sensor VL53L0X reiniciado.");
}

void VL53L0X::printLastReading() const {
	Serial.print("Ultima leitura: ");
	if (last_reading != 99999) {
		Serial.print(last_reading);
	}
	Serial.println(" mm");
}

void VL53L0X::printSensorInfo() const {
	Serial.println("Informações do sensor VL53L0X:");
	Serial.print("Ultima leitura: ");
	if (last_reading != 99999) {
		Serial.print(last_reading);
	}
	Serial.println(" mm");
}
