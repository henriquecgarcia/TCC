#pragma once

#include <Arduino.h>
#include "Encoder.h"
#include "PID.h"


class Motor {
private:
	unsigned long lastThink = 0;

	const uint8_t in1Pin, in2Pin, pwmPin;
	const uint8_t pwmChannel;

	Encoder* encoder;

	double targetRadS = rpmToRadS(100.0); // alvo em rad/s para controle
	double currentRPM = 0.0;
	double pidOutput = 0.0;

	PID pidRPM; // PID original para controle de RPM

	static constexpr unsigned long thinkInterval = 100; // ms entre updates; controle mais responsivo para células de 30 cm

	static constexpr uint32_t pwmFrequency = 5000; // 5 kHz
	static constexpr uint8_t pwmResolution = 8; // 8 bits = 0 a 255
	static constexpr int pwmMax = 255;

	int clamp(int value, int min, int max) {
		return (value < min) ? min : (value > max) ? max : value;
	}

	double rpmToRadS(double rpm) {
		return rpm * (M_PI / 30.0f); // converte RPM para rad/s
	}

	void writePWM(int value) {
		value = clamp(value, 0, pwmMax);
		ledcWrite(pwmChannel, value);
	}

public:
	Motor(int in1, int in2, int pwm, uint8_t channel, Encoder* enc, float kp_rpm = 1.0, float ki_rpm = 5.0, float kd_rpm = 0.0) :
		in1Pin(in1), in2Pin(in2), pwmPin(pwm), pwmChannel(channel), encoder(enc),
		pidRPM(kp_rpm, ki_rpm, kd_rpm, thinkInterval / 1000.0f) {}

	void begin();

	void setTunings(float kp, float ki, float kd);

	void setTargetRPM(double rpm);

	void forward();

	void backward();

	void stop();

	void update(double gyroError = 0.0);

	bool isMoving();

	void manualAddPID(int value);

	double getRPM() const;

	double getTargetRPM() const;

	double getTargetRadS() const;

	double getPIDOutput() const;

	bool isClockwise() const;
};