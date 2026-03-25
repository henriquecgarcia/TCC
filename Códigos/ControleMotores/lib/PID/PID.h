#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    PID(float Kp, float Ki, float Kd, float dt);

    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    void setTunning(float Kp, float Ki, float Kd);

    void setMaxMin(float max_v, float min_v);
    float compute(float setpoint, float measurement);
    float scaleToPWM(float output);

    void reset();

private:
    float _Kp, _Ki, _Kd;
    float _dt;
    float _integral, _prevError;
    float top_celing, bottom_floor;

    float clamp(float val, float min_val, float max_val);
};

#endif
