#include <aerial_robot/hardware/quadrotor_hardware.hpp>

QuadrotorHardware::QuadrotorHardware(std::vector<std::shared_ptr<Motor>> motors)
{
  motors_.resize(motors.size());

  for(int i = 0; i < motors.size();i ++)
    {
      motors_.at(i) = motors.at(i);
      motors_.at(i)->initialize();
      motors_.at(i)->setPwm(0.0);
    }
}

void QuadrotorHardware::update(BLA::Matrix<4, 1> actuator_input)
{
  for(int i = 0; i < motors_.size(); i++)
    {
      motors_.at(i)->setPwm(actuator_input(i));
    }
}
