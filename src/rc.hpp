// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the RC receiver

#include <SBUS.h>
#include "util.h"

// RC channels mapping:
extern int rollChannel;
extern int pitchChannel;
extern int throttleChannel;
extern int yawChannel;
extern int armedChannel;
extern int modeChannel;

extern double controlsTime; // time of the last controls update
extern float channelNeutral[16]; // first element NAN means not calibrated
extern float channelMax[16];

void setupRC();

bool readRC();

void normalizeRC();

void calibrateRC();

void printRCCal();
