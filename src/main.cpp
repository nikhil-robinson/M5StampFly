// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "variable.hpp"



void setup() {
	USBSerial.begin(SERIAL_BAUDRATE);
	print("Initializing flix");
	disableBrownOut();
	setupParameters();
	// setupLED();
	setupMotors();
	// setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setupRC();
	// setLED(false);
	print("Initializing complete");
}

void loop() {
	readIMU();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
}
