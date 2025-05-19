#pragma once

#include <aerial_robot/state_estimation/position/extended_kalman_filter.hpp>
#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>
#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>
#include <memory>
#include <BasicLinearAlgebra.h>

class PositionEstimator {
   public:
    PositionEstimator(std::shared_ptr<AttitudeEstimator> attitude_estimator,
                      std::shared_ptr<AltitudeEstimator> altitude_estimator)
        : attitude_estimator_(attitude_estimator), altitude_estimator_(altitude_estimator) {
        initialize();
    }

    ~PositionEstimator() = default;

    void initialize() {
        ekf_.initialize();
        ekf_.setRotationMatrix(attitude_estimator_->getRotationMatrix());
    }

    void update(float x_vel, float y_vel, float tof, float acc_x, float acc_y, float acc_z, float omega_x,
                float omega_y, float omega_z) {
        ekf_.setRotationMatrix(attitude_estimator_->getRotationMatrix());
        ekf_.update(x_vel, y_vel, tof, acc_x, acc_y, acc_z, omega_x, omega_y, omega_z);
    }

    void reset();

    BLA::Matrix<3, 1> getPos() {
        return ekf_.getPos();
    }
    BLA::Matrix<3, 1> getVel() {
        return ekf_.getVel();
    }

   private:
    std::shared_ptr<AttitudeEstimator> attitude_estimator_;
    std::shared_ptr<AltitudeEstimator> altitude_estimator_;
    ExtendedKalmanFilter ekf_;
};
