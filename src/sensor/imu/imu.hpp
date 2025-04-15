#pragma once

#include <BasicLinearAlgebra.h>

class Imu
{
public:
  Imu();
  ~Imu() = default;

  virtual void initialize() {};
  void update();
  BLA::Matrix<3, 1> getAcc() {return acc_data_;}
  BLA::Matrix<3, 1> getGyro() {return gyro_data_;}

protected:
  virtual void readImuData(){};
  BLA::Matrix<3, 1> gyro_data_;
  BLA::Matrix<3, 1> acc_data_;
};
