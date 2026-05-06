#include "PID.h"

PID::PID(float kp, float ki, float kd, float dt)
	: _kp(kp), _ki(ki), _kd(kd), _dt(dt > 0.0f ? dt : 0.001f), _integral(0.0f), _prevError(0.0f), _minOut(-255.0f), _maxOut(255.0f) {}

void PID::setTunings(float kp, float ki, float kd) {
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

void PID::setLimits(float minOut, float maxOut) {
	if (minOut > maxOut) return;
	_minOut = minOut;
	_maxOut = maxOut;
	_integral = clamp(_integral, _minOut, _maxOut);
}

float PID::compute(float setpoint, float measurement) {
	const float error = setpoint - measurement;
	_integral = clamp(_integral + error * _dt, _minOut, _maxOut);
	const float derivative = (error - _prevError) / _dt;
	_prevError = error;
	return clamp((_kp * error) + (_ki * _integral) + (_kd * derivative), _minOut, _maxOut);
}

void PID::reset() {
	_integral = 0.0f;
	_prevError = 0.0f;
}

float PID::clamp(float v, float lo, float hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}
