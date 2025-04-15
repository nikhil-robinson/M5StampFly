#include <aerial_robot/wrench_allocation/quadrotor_wrench_allocation.hpp>

QuadrotorWrenchAllocation::QuadrotorWrenchAllocation(std::vector<std::shared_ptr<Motor>> motors)
{
  motors_.resize(motors.size());
  actuator_input_ = {0.0, 0.0, 0.0, 0.0};

  // make thrust->wrench conversion matrix once
  for(int i = 0; i < motors.size(); i++)
    {
      motors_.at(i) = motors.at(i);
      BLA::Matrix<3, 1> motor_pos = motors_.at(i)->getPos();
      BLA::Matrix<3, 1> motor_axis = motors_.at(i)->getAxis();
      float motor_sigma = motors_.at(i)->getSigma();

      q_mat_(0, i) = motor_axis(2); // z_axis

      BLA::Matrix<3, 1> p_cross_u = CrossProduct(motor_pos, motor_axis);
      for(int j = 0; j < 3; j++)
        {
          q_mat_(1 + j, i) = p_cross_u(j) + motor_sigma * motor_axis(j); // roll elemnt
        }
    }

  // make wrench->thrust conversion matrix once
  q_mat_inv_ = Inverse(q_mat_);

  USBSerial.print("q_matrix: ");
  USBSerial.print(q_mat_);
  USBSerial.print("\n");
  USBSerial.print("q_matrix_inverse: ");
  USBSerial.print(q_mat_inv_);
  USBSerial.print("\n");
}

void QuadrotorWrenchAllocation::update(BLA::Matrix<4, 1> control_input)
{
  actuator_input_ = q_mat_inv_ * control_input;
}
