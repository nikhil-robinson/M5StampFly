#pragma once

#include <vector>
#include <aerial_robot/motor/motor.hpp>
#include <memory>

class QuadrotorHardware
{
public:
  QuadrotorHardware(std::vector<std::shared_ptr<Motor>> motors);
  ~QuadrotorHardware() = default;

  void update(BLA::Matrix<4, 1> actuator_input);

private:
  float clamp(float value, float min_val, float max_val) {return std::max(min_val, std::min(value, max_val));}

  std::vector<std::shared_ptr<Motor>> motors_;
};
