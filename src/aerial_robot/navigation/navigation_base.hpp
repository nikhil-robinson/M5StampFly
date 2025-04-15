#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <aerial_robot/state_estimation/state_estimator.hpp>

enum flight_state
  {
   ARM_OFF_STATE,
   TAKEOFF_STATE,
   LAND_STATE
  };

class BaseNavigator
{
public:
  BaseNavigator(std::shared_ptr<Odometry> odom):
    odom_(odom)
  {
    target_pos_ = {0.0, 0.0, 0.0};
    target_vel_ = {0.0, 0.0, 0.0};
    target_acc_ = {0.0, 0.0, 0.0};
    target_rpy_ = {0.0, 0.0, 0.0};
    target_omega_ = {0.0, 0.0, 0.0};
    target_ang_acc_ = {0.0, 0.0, 0.0};
  }
  ~BaseNavigator() = default;

  virtual void initialize(){};
  virtual void update(){};

  int getFlightState() {return flight_state_;}

  float getTargetPosZ() {return target_pos_(2);}
  BLA::Matrix<3, 1> getTargetRpy() {return target_rpy_;}
  float getTargetRoll() {return target_rpy_(0);}
  float getTargetPitch() {return target_rpy_(1);}
  float getTargetYaw() {return target_rpy_(2);}

protected:
  std::shared_ptr<Odometry> odom_;
  int flight_state_ = ARM_OFF_STATE;
  BLA::Matrix<3, 1> target_pos_;
  BLA::Matrix<3, 1> target_vel_;
  BLA::Matrix<3, 1> target_acc_;
  BLA::Matrix<3, 1> target_rpy_;
  BLA::Matrix<3, 1> target_omega_;
  BLA::Matrix<3, 1> target_ang_acc_;
};
