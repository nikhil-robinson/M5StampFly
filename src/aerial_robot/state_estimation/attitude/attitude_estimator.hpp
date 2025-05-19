#pragma once

#include <sensor/imu/imu.hpp>
#include <sensor/mag/mag.hpp>
#include <memory>
#include <BasicLinearAlgebra.h>
#include <aerial_robot/state_estimation/attitude/complementary_ahrs.hpp>
#include <utils/lpf/lpf.hpp>

class AttitudeEstimator {
   public:
    AttitudeEstimator(std::shared_ptr<Imu> imu, std::shared_ptr<Magnetmeter> magnetmeter);
    ~AttitudeEstimator() = default;

    void update();

    std::shared_ptr<Imu> imu_;
    std::shared_ptr<Magnetmeter> magnetmeter_;
    ComplementaryAHRS complementary_filter_;

    BLA::Matrix<3, 1> getAcc() {
        return complementary_filter_.getAcc();
    }
    BLA::Matrix<3, 1> getFilteredAcc() {
        return filtered_acc_;
    }
    float getFilteredAccX() {
        return filtered_acc_(0);
    }
    float getFilteredAccY() {
        return filtered_acc_(1);
    }
    float getFilteredAccZ() {
        return filtered_acc_(2);
    }
    BLA::Matrix<3, 1> getGyro() {
        return complementary_filter_.getGyro();
    }
    BLA::Matrix<3, 1> getRpy() {
        return complementary_filter_.getRpy();
    }
    BLA::Matrix<3, 3> getRotationMatrix();

   private:
    float invSqrt(float x);

    Filter acc_x_lpf_;
    Filter acc_y_lpf_;
    Filter acc_z_lpf_;

    BLA::Matrix<3, 1> filtered_acc_;
};
