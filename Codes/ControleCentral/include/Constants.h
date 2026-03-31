#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

#define MOTOR_CONTROLER_ESP32_ADDR 0x42

enum Movement {
	MOVEMENT_FORWARD = 0,
	MOVEMENT_BACKWARDS = 1,
	MOVEMENT_TURN_LEFT = 2,
	MOVEMENT_TURN_RIGHT = 3,
	MOVEMENT_STOPPED = 4
};

typedef struct {
	int16_t targetRadSRight;
	int16_t targetRadSLeft;
} MotorCommand;

typedef struct {
    int16_t radSLeft;
    int16_t radSRight;
    int16_t pwmLeft;
    int16_t pwmRight;
} MotorStatus;

#endif