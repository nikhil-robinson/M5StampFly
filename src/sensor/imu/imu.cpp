#include <sensor/imu/imu.hpp>

Imu::Imu()
{
  gyro_data_ = {0.0, 0.0, 0.0};
  acc_data_ = {0.0, 0.0, 0.0};
}

void Imu::update()
{
  readImuData();
}
