#include <sensor/imu/imu.hpp>

Imu::Imu() {
    gyro_data_ = {0.0, 0.0, 0.0};
    acc_data_  = {0.0, 0.0, 0.0};
}

void Imu::update() {
    if (micros() - last_update_time_ < SENSOR::IMU_UPDATE_DU * 1000 * 1000) return;

    readImuData();
    last_update_time_ = micros();

    gyro_data_ = gyro_data_raw_ - gyro_bias_;
    acc_data_  = acc_data_raw_ - acc_bias_;
}

void Imu::calibrate() {
    uint32_t duration = 3 * 1000 * 1000;  // 3 seconds
    uint32_t start_t  = micros();
    int N             = 0;
    while (micros() - start_t < duration) {
        readImuData();
        gyro_bias_ += gyro_data_raw_;
        acc_bias_ += acc_data_raw_;
        N++;
        delay(SENSOR::IMU_UPDATE_DU * 1000);
    }
    gyro_bias_ = gyro_bias_ / (float)N;
    acc_bias_  = acc_bias_ / (float)N - BLA::Matrix<3, 1>{0.0, 0.0, 1.0};

    USBSerial.printf("Gyro bias: %f %f %f\n", gyro_bias_(0), gyro_bias_(1), gyro_bias_(2));
    USBSerial.printf("Acc bias: %f %f %f\n", acc_bias_(0), acc_bias_(1), acc_bias_(2));
}
