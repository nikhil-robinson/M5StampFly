#include "variable.hpp"


double t = NAN; // current step time, s
float dt; // time delta from previous step, s
int16_t channels[16]; // raw rc channels
float controls[16]; // normalized controls in range [-1..1] ([0..1] for throttle)
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [-1..1]