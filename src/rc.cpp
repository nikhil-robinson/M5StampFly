// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the RC receiver

#include <sbus.h>
#include "util.h"
#include "variable.hpp"

#if 1

// SbusTx RC(Serial2); // NOTE: Use RC(Serial2, 16, 17) if you use the old UART2 pins

// RC channels mapping:
int rollChannel = 0;
int pitchChannel = 1;
int throttleChannel = 2;
int yawChannel = 3;
int armedChannel = 4;
int modeChannel = 5;

double controlsTime; // time of the last controls update
float channelNeutral[16] = {NAN}; // first element NAN means not calibrated
float channelMax[16];

void setupRC() {
	print("Setup RC\n");
	// RC.begin();
}

bool readRC() {

	return false;
}

void normalizeRC() {

}

void calibrateRC() {

}

void printRCCal() {

}
#endif