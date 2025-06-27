#include "LED.h"

LED::LED(uint8_t pin) : pin(pin) {}

void LED::setup() {
	pinMode(pin, OUTPUT);
	off();  // já garante pino em LOW e state = false
}

void LED::on() {
	digitalWrite(pin, HIGH);
	state = true;
}

void LED::off() {
	digitalWrite(pin, LOW);
	state = false;
}

bool LED::isOn() const {
	return state;
}

void LED::toggle() {
#ifdef DEBUG_PRINTS
	Serial.print("Toggling LED ");
	Serial.println(pin);
#endif
	if (state) off();
	else on();
}