#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>
#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>

class Odometry
{
public:
  Odometry(std::shared_ptr<AttitudeEstimator> attitude_estimator,
           std::shared_ptr<AltitudeEstimator> altitude_estimator);
  ~Odometry() = default;

  void update();

  BLA::Matrix<3, 1> getPos() {return pos_;}
  float getPosZ() {return pos_(2);}
  BLA::Matrix<3, 1> getVel() {return vel_;}
  BLA::Matrix<3, 1> getAcc() {return acc_;}
  BLA::Matrix<3, 1> getRpy() {return rpy_;}
  float getRoll() {return rpy_(0);}
  float getPitch() {return rpy_(1);}
  float getYaw() {return rpy_(2);}
  BLA::Matrix<3, 1> getOmega() {return omega_;}
  float getOmegaX() {return omega_(0);}
  float getOmegaY() {return omega_(1);}
  float getOmegaZ() {return omega_(2);}
  BLA::Matrix<3, 1> getAngAcc() {return ang_acc_;}

private:
  std::shared_ptr<AttitudeEstimator> attitude_estimator_;
  std::shared_ptr<AltitudeEstimator> altitude_estimator_;

  BLA::Matrix<3, 1> pos_;
  BLA::Matrix<3, 1> vel_;
  BLA::Matrix<3, 1> acc_;
  BLA::Matrix<3, 1> rpy_;
  BLA::Matrix<3, 1> omega_;
  BLA::Matrix<3, 1> ang_acc_;
};
