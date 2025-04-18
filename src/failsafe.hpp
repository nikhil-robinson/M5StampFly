// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Fail-safe functions
#pragma once


#define RC_LOSS_TIMEOUT 0.2
#define DESCEND_TIME 3.0 // time to descend from full throttle to zero


void failsafe() ;

// Prevent arming without zero throttle input
void armingFailsafe();

// RC loss failsafe
void rcLossFailsafe();

// Smooth descend on RC lost
void descend() ;
