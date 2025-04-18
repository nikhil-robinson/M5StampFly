// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Motors output control using MOSFETs
// In case of using ESCs, change PWM_STOP, PWM_MIN and PWM_MAX to appropriate values in μs, decrease PWM_FREQUENCY (to 400)
#pragma once

#include "util.h"

#define MOTOR_0_PIN 12 // rear left
#define MOTOR_1_PIN 13 // rear right
#define MOTOR_2_PIN 14 // front right
#define MOTOR_3_PIN 15 // front left

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 12
#define PWM_STOP 0
#define PWM_MIN 0
#define PWM_MAX 1000000 / PWM_FREQUENCY

// Motors array indexes:
extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;

void setupMotors() ;

int getDutyCycle(float value);

void sendMotors() ;

bool motorsActive() ;

void testMotor(uint8_t n);
