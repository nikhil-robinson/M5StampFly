#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <utils/pid/pid_controller.hpp>
#include <aerial_robot/state_estimation/state_estimator.hpp>
#include <aerial_robot/navigation/navigation_base.hpp>
#include <config.h>

class UnderActuatedFlightController
{
public:
  UnderActuatedFlightController(std::shared_ptr<Odometry> odom,
                                std::shared_ptr<BaseNavigator> navigator);
  ~UnderActuatedFlightController() = default;

  void update();
  BLA::Matrix<4, 1> getControlInput() {return control_input_;}

private:
  std::shared_ptr<Odometry> odom_;
  std::shared_ptr<BaseNavigator> navigator_;
  BLA::Matrix<4, 1> control_input_;
  PIDController z_pid_;
  PIDController roll_pid_;
  PIDController pitch_pid_;
  PIDController yaw_pid_;
};
