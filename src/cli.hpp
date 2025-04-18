// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Implementation of command line interface
#pragma once

#include "pid.h"
#include "vector.h"
#include "util.h"
#include "Arduino.h"




void print(const char* format, ...);

void pause(float duration);

void doCommand(String str, bool echo = false);

void handleInput();