// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Parameters storage in flash memory
#pragma once


#include <Preferences.h>


extern Preferences storage;

struct Parameter {
	const char *name;
	float *variable;
	float value; // cache
};

extern Parameter parameters[];

void setupParameters();
int parametersCount();

const char *getParameterName(int index);

float getParameter(int index);

float getParameter(const char *name);

bool setParameter(const char *name, const float value);

void syncParameters();
void printParameters() ;

void resetParameters();