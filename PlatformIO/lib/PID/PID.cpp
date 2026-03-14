#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float dt)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _dt(dt), _integral(0.0f), _prevError(0.0f), top_celing(255.0f), bottom_floor(0.0f) {
    if (_dt <= 0.0f) {
        _dt = 1e-3f; // 1 ms mínimo
    }
}

void PID::setKp(float Kp) { _Kp = Kp; }
void PID::setKi(float Ki) { _Ki = Ki; }
void PID::setKd(float Kd) { _Kd = Kd; }

void PID::setTunning(float Kp, float Ki, float Kd) {
    setKp(Kp);
    setKi(Ki);
    setKd(Kd);
}

void PID::setMaxMin(float max_v, float min_v) {
    top_celing = max_v;
    bottom_floor = min_v;

    if (bottom_floor > top_celing) {
        float temp = bottom_floor;
        bottom_floor = top_celing;
        top_celing = temp;
    }
}

float PID::compute(float setpoint, float measurement) {
    float error = setpoint - measurement;

    _integral += error * _dt;
    _integral = clamp(_integral, bottom_floor, top_celing);

    float derivative = (error - _prevError) / _dt;
    _prevError = error;

    float output = _Kp * error + _Ki * _integral + _Kd * derivative;
    output = clamp(output, bottom_floor, top_celing);
    return output;
}

float PID::scaleToPWM(float output) {
    const float range = top_celing - bottom_floor;
    if (range <= 0.0f) {
        return 0.0f; // Evita divisão por zero
    }
    float scaled = (output - bottom_floor) * (255.0f / (top_celing - bottom_floor));
    return clamp(scaled, 0.0f, 255.0f);
}

void PID::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

float PID::clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
