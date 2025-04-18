// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// In-RAM logging

#include "vector.h"

#define LOG_RATE 100
#define LOG_DURATION 10
#define LOG_PERIOD 1.0 / LOG_RATE
#define LOG_SIZE LOG_DURATION * LOG_RATE

extern float tFloat;
extern Vector attitudeEuler;
extern Vector attitudeTargetEuler;

struct LogEntry {
	const char *name;
	float *value;
};



void prepareLogData();

void logData();
void dumpLog();
