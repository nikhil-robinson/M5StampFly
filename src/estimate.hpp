// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Attitude estimation from gyro and accelerometer
#pragma once

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

#define WEIGHT_ACC 0.003
#define RATES_LFP_ALPHA 0.2 // cutoff frequency ~ 40 Hz

void estimate() ;

void applyGyro() ;

void applyAcc();
