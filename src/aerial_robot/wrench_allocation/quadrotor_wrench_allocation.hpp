#pragma once

#include <aerial_robot/motor/motor.hpp>
#include <vector>
#include <BasicLinearAlgebra.h>
#include <memory>

class QuadrotorWrenchAllocation
{
public:
  QuadrotorWrenchAllocation(std::vector<std::shared_ptr<Motor>> motors);
  ~QuadrotorWrenchAllocation() = default;

  void update(BLA::Matrix<4, 1> control_input);
  BLA::Matrix<4, 1> getActuatorInput() {return actuator_input_;}

private:
  std::vector<std::shared_ptr<Motor>> motors_;
  BLA::Matrix<4, 4> q_mat_;
  BLA::Matrix<4, 4> q_mat_inv_;
  BLA::Matrix<4, 1> actuator_input_;
};
