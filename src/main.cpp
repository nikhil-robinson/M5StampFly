#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "variable.hpp"
#include "led.hpp"
#include "buzzer.h"
#include "button.hpp"


void setup() {
	USBSerial.begin(SERIAL_BAUDRATE);
	print("Initializing flix");
	led_init();
    esp_led(0x110000, 1);
    onboard_led1(WHITE, 1);
    onboard_led2(WHITE, 1);
    led_show();
    led_show();
    led_show();
	init_button();
	setup_pwm_buzzer();
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
	start_tone();
}

void loop() {
	led_drive();
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
