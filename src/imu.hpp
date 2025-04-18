#include <SPI.h>
#include "lpf.h"
#include "util.h"
#include "bmi_imu.hpp"
#include "variable.hpp"

extern Vector accBias;
extern Vector gyroBias;
extern Vector accScale;

void setupIMU();

void configureIMU();

void readIMU();

void rotateIMU(Vector& data);

void calibrateGyroOnce();
void calibrateAccel();

void calibrateAccelOnce();

void printIMUCal();