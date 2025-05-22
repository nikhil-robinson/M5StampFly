#include <sensor/imu/imu_bmi270.hpp>
#include <sensor/mag/mag_bmm150.hpp>
#include "calibrate.h"
#include <i2c.hpp>
#include <spi_s3.hpp>

static std::shared_ptr<Magnetmeter> mag;
static std::shared_ptr<Imu> imu;

int loopcount = 0;

void calibration_setup(void) {
    USBSerial.println(F("Sensor Lab - IMU Calibration!"));
    spi_init();

    Wire1.end();
    delay(10);
    Wire1.begin(I2C::SDA_PIN, I2C::SCL_PIN, 400000UL);
    // i2c_master_init();
    i2c_scan();

    mag = std::make_shared<MagnetmeterBMM150>();
    imu = std::make_shared<ImuBMI270>();
    mag->initialize();
    imu->initialize();
    imu->calibrate();
}

void calibration_loop() {
    // 'Raw' values to match expectation of MOtionCal
    mag->update();
    imu->update();
    BLA::Matrix<3, 1> mag_data = mag->getRawMag();

    USBSerial.print("Raw:");
    USBSerial.print(int(imu->getAccX()*8192/9.8));
    USBSerial.print(",");
    USBSerial.print(int(imu->getAccY()*8192/9.8));
    USBSerial.print(",");
    USBSerial.print(int(imu->getAccZ()*8192/9.8));
    USBSerial.print(",");
    USBSerial.print(int(imu->getGyroX() * DPS20002RAD * 16));
    USBSerial.print(",");
    USBSerial.print(int(imu->getGyroY()* DPS20002RAD * 16));
    USBSerial.print(",");
    USBSerial.print(int(imu->getGyroZ()* DPS20002RAD * 16));
    USBSerial.print(",");
    USBSerial.print(int(mag_data(0) *10));
    USBSerial.print(",");
    USBSerial.print(int(mag_data(1) *10));
    USBSerial.print(",");
    USBSerial.print(int(mag_data(2) *10));
    USBSerial.println("");
    delay(10);

    // // unified data
    USBSerial.print("Uni:");
    USBSerial.print(imu->getAccX());
    USBSerial.print(",");
    USBSerial.print(imu->getAccY());
    USBSerial.print(",");
    USBSerial.print(imu->getAccZ());
    USBSerial.print(",");
    USBSerial.print(imu->getGyroX(),4);
    USBSerial.print(",");
    USBSerial.print(imu->getGyroY(),4);
    USBSerial.print(",");
    USBSerial.print(imu->getGyroZ(),4);
    USBSerial.print(",");
    USBSerial.print(mag_data(0));
    USBSerial.print(",");
    USBSerial.print(mag_data(1));
    USBSerial.print(",");
    USBSerial.print(mag_data(2));
    USBSerial.println("");
    delay(10);
}