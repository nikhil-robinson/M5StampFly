#include <aerial_robot/motor/motor.hpp>

Motor::Motor(int motor_pin, int motor_channel,
             BLA::Matrix<3, 1> pos, BLA::Matrix<3, 1> axis,
             float sigma):
  motor_pin_(motor_pin),
  motor_channel_(motor_channel),
  sigma_(sigma)
{
  for(int i = 0; i < 3; i++)
    {
      pos_(i) = pos(i);
      axis_(i) = axis(i);
    }
}


void Motor::initialize()
{
  USBSerial.printf("initialize Motor %d connected to %d pin\n", motor_channel_, motor_pin_);
  ledcSetup(motor_channel_, motor_freq_, motor_resolution_);
  ledcAttachPin(motor_pin_, motor_channel_);
  USBSerial.printf("Motor %d init done\n", motor_channel_);
}

void Motor::setPwm(float duty)
{
  ledcWrite(motor_channel_,  (uint32_t)(255 * duty));
}
