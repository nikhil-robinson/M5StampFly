#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>
#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>
#include <aerial_robot/state_estimation/position/position_estimator.hpp>

class Odometry {
   public:
    Odometry(std::shared_ptr<AttitudeEstimator> attitude_estimator,
             std::shared_ptr<AltitudeEstimator> altitude_estimator,
             std::shared_ptr<PositionEstimator> position_estimator);
    ~Odometry() = default;

    void update();

    BLA::Matrix<3, 1> getPos() {
        return position_estimator_->getPos();
    }
    float getPosZ() {
        return altitude_estimator_->getAltitude();
        ;
    }
    BLA::Matrix<3, 1> getVel() {
        return position_estimator_->getVel();
    }
    float getVelZ() {
        return altitude_estimator_->getVelocity();
    }
    //  BLA::Matrix<3, 1> getAcc() {return acc_;}
    BLA::Matrix<3, 1> getRpy() {
        return attitude_estimator_->getRpy();
    }
    float getRoll() {
        return getRpy()(0);
    }
    float getPitch() {
        return getRpy()(1);
    }
    float getYaw() {
        return getRpy()(2);
    }
    BLA::Matrix<3, 1> getOmega() {
        return attitude_estimator_->getGyro();
    }
    float getOmegaX() {
        return getOmega()(0);
    }
    float getOmegaY() {
        return getOmega()(1);
    }
    float getOmegaZ() {
        return getOmega()(2);
    }
    // BLA::Matrix<3, 1> getAngAcc() {return ang_acc_;}

   private:
    std::shared_ptr<AttitudeEstimator> attitude_estimator_;
    std::shared_ptr<AltitudeEstimator> altitude_estimator_;
    std::shared_ptr<PositionEstimator> position_estimator_;
};
