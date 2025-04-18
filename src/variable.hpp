#ifndef VARIABLE_HPP
#define VARIABLE_HPP

#include "util.h"
#include "time.hpp"
#include "rc.hpp"
#include "parameters.hpp"
#include "motors.hpp"
#include "mavlink.hpp"
#include "lpf.h"
#include "failsafe.hpp"
#include "estimate.hpp"
#include "control.hpp"
#include "cli.hpp"
#include "bmi_imu.hpp"
#include "wifi.hpp"
#include "vector.h"
#include "Arduino.h"
#include "quaternion.h"
#include "pid.h"
#include "util.h"
#include "imu.hpp"
#include "log.hpp"

#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 1

extern double t;
extern float dt; // time delta from previous step, s
extern int16_t channels[16]; // raw rc channels
extern float controls[16]; // normalized controls in range [-1..1] ([0..1] for throttle)
extern Vector gyro; // gyroscope data
extern Vector acc; // accelerometer data, m/s/s
extern Vector rates; // filtered angular rates, rad/s
extern Quaternion attitude; // estimated attitude
extern bool landed; // are we landed and stationary
extern float motors[4]; // normalized motors thrust in range [-1..1]

#endif // VARIABLE_HPP