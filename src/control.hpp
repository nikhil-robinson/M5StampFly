// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Flight control
#pragma once


#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

#define PITCHRATE_P 0.05
#define PITCHRATE_I 0.2
#define PITCHRATE_D 0.001
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3
#define ROLL_P 4.5
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3
#define PITCHRATE_MAX radians(360)
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300)
#define TILT_MAX radians(30)

#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

enum { MANUAL, ACRO, STAB, USER };
enum { YAW, YAW_RATE };
extern bool armed;

extern PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
extern PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
extern PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
extern PID rollPID(ROLL_P, ROLL_I, ROLL_D);
extern PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
extern PID yawPID(YAW_P, 0, 0);
extern Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
extern float tiltMax;

extern Quaternion attitudeTarget;
extern Vector ratesTarget;
extern Vector torqueTarget;
extern float thrustTarget;


void control();
void interpretRC();

void controlAttitude();

void controlRate();

void controlTorque();

const char* getModeName();
