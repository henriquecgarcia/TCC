#include <Arduino.h>
#include "Motor.h"

void Motor::begin() {
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);

	ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
	ledcAttachPin(pwmPin, pwmChannel);
	writePWM(0);

	encoder->begin();
	encoder->reset();

	stop();

	// limites RPM em rad/s
	pidRPM.setMaxMin(rpmToRadS(200.0), 0.0);
}

void Motor::setTunings(float kp, float ki, float kd) {
	pidRPM.setTunning(kp, ki, kd);
}

void Motor::setTargetRPM(double rpm) {
	targetRadS = rpmToRadS(rpm);
}

void Motor::forward() {
	encoder->reset();
	stop();

	digitalWrite(in1Pin, HIGH);
	digitalWrite(in2Pin, LOW);

	targetRadS = rpmToRadS(100.0);
}

void Motor::backward() {
	encoder->reset();
	stop();

	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, HIGH);

	targetRadS = rpmToRadS(100.0);
}

void Motor::stop() {
	// Primeiro corta PWM, depois coloca a ponte H em neutro.
	// Essa ordem reduz a chance de pulso residual no L298N ao parar.
	writePWM(0);
	digitalWrite(in1Pin, LOW);
	digitalWrite(in2Pin, LOW);

	encoder->reset();

	targetRadS = 0.0;
	pidRPM.reset();
	currentRPM = 0.0;
	pidOutput = 0.0;

	writePWM(0);
	lastThink = millis();
}

void Motor::update(double gyroError) {
	unsigned long now = millis();

	if (now - lastThink < thinkInterval) return;

	lastThink = now;

	currentRPM = encoder->getRPM(10, thinkInterval / 1000.0);

	// controle de RPM com compensação de giro, se houver
	float targetRadS = this->targetRadS + gyroError;
	float currentRadS = rpmToRadS(currentRPM);

	float rpmControl = pidRPM.compute(targetRadS, currentRadS);
	rpmControl = pidRPM.scaleToPWM(rpmControl); // converte para valor de PWM

	pidOutput = rpmControl;
	pidOutput = clamp(pidOutput, 0.0, 255.0);

	writePWM((int)pidOutput);
	// writePWM((int)150);

	encoder->reset();

	float effectiveTargetRadS = targetRadS + gyroError;

	if (effectiveTargetRadS < 0.001) {
		this->targetRadS = rpmToRadS(100.0);
	}
}

bool Motor::isMoving() {
	return (pidOutput > 1.0) && (targetRadS > 0.0);
}

void Motor::manualAddPID(int value) {
	pidOutput += value;
	pidOutput = clamp(pidOutput, 0.0, 255.0);

	writePWM((int)pidOutput);
}

double Motor::getRPM() const {
	return currentRPM;
}

double Motor::getTargetRPM() const {
	return targetRadS * (30.0 / M_PI); // converte de volta para RPM
}

double Motor::getTargetRadS() const {
	return targetRadS;
}

double Motor::getPIDOutput() const {
	return pidOutput;
}

bool Motor::isClockwise() const {
	return encoder->isClockwise();
}
