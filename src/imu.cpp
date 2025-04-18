#include <SPI.h>
#include "lpf.h"
#include "util.h"
#include "bmi_imu.hpp"
#include "variable.hpp"

Vector accBias;
Vector gyroBias;
Vector accScale(1, 1, 1);

void setupIMU() {
	print("Setup IMU\n");
    imu_init();
    configureIMU();
}

void configureIMU() 
{
    imu_setAccelRange(BMI2_ACC_RANGE_4G);
	imu_setGyroRange(BMI2_GYR_RANGE_2000);
}

void readIMU() {
    imu_update();
    gyro.x = imu_get_gyro_x();
    gyro.y = imu_get_gyro_y();
    gyro.z = imu_get_gyro_z();
    acc.x = imu_get_acc_x();
    acc.y = imu_get_acc_y();
    acc.z = imu_get_acc_z();
	calibrateGyroOnce();
	// apply scale and bias
	acc = (acc - accBias) / accScale;
	gyro = gyro - gyroBias;

    rotateIMU(acc);
	rotateIMU(gyro);
}

void rotateIMU(Vector& data) {
	data = Vector(data.x, data.y, -data.z);
}

void calibrateGyroOnce() {
	static float landedTime = 0;
	landedTime = landed ? landedTime + dt : 0;
	if (landedTime < 2) return; // calibrate only if definitely stationary

	static LowPassFilter<Vector> gyroCalibrationFilter(0.001);
	gyroBias = gyroCalibrationFilter.update(gyro);
}

void calibrateAccel() {
	print("Calibrating accelerometer\n");
	imu_setAccelRange(BMI2_ACC_RANGE_2G); // the most sensitive mode

	print("Place level [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("Place nose up [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("Place nose down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("Place on right side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("Place on left side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("Place upside down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();

	printIMUCal();
	print("✓ Calibration done!\n");
	configureIMU();
}

void calibrateAccelOnce() {
	const int samples = 1000;
	static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
	static Vector accMin(INFINITY, INFINITY, INFINITY);

	// Compute the average of the accelerometer readings
	acc = Vector(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		Vector sample;
        imu_update();
        sample.x = imu_get_acc_x();
        sample.y = imu_get_acc_y();
        sample.z = imu_get_acc_z();
		acc = acc + sample;
	}
	acc = acc / samples;

	// Update the maximum and minimum values
	if (acc.x > accMax.x) accMax.x = acc.x;
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;
	// Compute scale and bias
	accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}

void printIMUCal() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}
