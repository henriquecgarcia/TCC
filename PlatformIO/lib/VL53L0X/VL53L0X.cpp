#include "VL53L0X.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

void VL53L0X::setup() {
	if (!sensor.begin()) {
		#ifdef DEBUG_PRINTS
		Serial.println("Falha ao encontrar o sensor VL53L0X");
		Serial.println("Reiniciando ESP32...");
		#endif
		pinMode(BUILTIN_LED, OUTPUT);
		digitalWrite(BUILTIN_LED, HIGH); // acende o LED para indicar erro
		delay(1000); // espera 1 segundo antes de reiniciar
		digitalWrite(BUILTIN_LED, LOW); // apaga o LED
		ESP.restart(); // reinicia o ESP32 se o sensor não for encontrado
		return;
	}
	#ifdef DEBUG_PRINTS
	Serial.println("Sensor VL53L0X encontrado!");
	#endif
}

VL53L0X::VL53L0X() : lastRead(0), last_reading(0) {
	// sensor.startMeasurement();
}

int VL53L0X::loop() {
	unsigned long now = millis();
	if (now - lastRead < readInterval) {
		return last_reading;
	}
	lastRead = now;

	VL53L0X_RangingMeasurementData_t measure;
	sensor.rangingTest(&measure, false);
	if (measure.RangeStatus != 4) { // se não estiver fora de alcance
		last_reading = measure.RangeMilliMeter;
	} else {
		last_reading = 99999;
	}
	return last_reading;
}

void VL53L0X::reset() {
	lastRead = 0;
	last_reading = 0;
	#ifdef DEBUG_PRINTS
	Serial.println("Sensor VL53L0X reiniciado.");
	#endif
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
