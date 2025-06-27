#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED {
private:
	const uint8_t pin;
	bool state = false;

public:
	// Impede cópia acidental
	LED(const LED&) = delete;
	LED& operator=(const LED&) = delete;

	// Construtor com initializer list
	LED(uint8_t pin);

	void setup();
	void on();
	void off();
	bool isOn() const;
	void toggle();
};

#endif