#pragma once

#include <config.h>
#include <BasicLinearAlgebra.h>

class Imu {
   public:
    Imu();
    ~Imu() = default;

    virtual void initialize(){};
    void update();
    void calibrate();
    BLA::Matrix<3, 1> getAcc() {
        return acc_data_;
    }
    float getAccX() {
        return acc_data_(0);
    }
    float getAccY() {
        return acc_data_(1);
    }
    float getAccZ() {
        return acc_data_(2);
    }
    BLA::Matrix<3, 1> getGyro() {
        return gyro_data_;
    }
    float getGyroX() {
        return gyro_data_(0);
    }
    float getGyroY() {
        return gyro_data_(1);
    }
    float getGyroZ() {
        return gyro_data_(2);
    }

   protected:
    uint32_t last_update_time_ = 0;
    virtual void readImuData(){};
    BLA::Matrix<3, 1> gyro_data_;
    BLA::Matrix<3, 1> gyro_bias_ = {0.0, 0.0, 0.0};
    BLA::Matrix<3, 1> gyro_data_raw_;
    BLA::Matrix<3, 1> acc_data_;
    BLA::Matrix<3, 1> acc_bias_ = {0.0, 0.0, 0.0};
    BLA::Matrix<3, 1> acc_data_raw_;
};
