#pragma once

#include <BasicLinearAlgebra.h>

class Magnetmeter
{
public:
  Magnetmeter() {};
  ~Magnetmeter() = default;

  virtual void initialize();
  void setOffsetVector(BLA::Matrix<3, 1> offset) {offset_ = offset;}
  void setScaliingMatrix(BLA::Matrix<3, 3> scaling_matrix) {scaling_matrix_ = scaling_matrix;}
  BLA::Matrix<3, 1> getMag() {return mag_data_;}
  BLA::Matrix<3, 1> getRawMag() {return raw_mag_data_;}
  void update();

protected:
  virtual void readMagData(){};
  BLA::Matrix<3, 1> raw_mag_data_;
  BLA::Matrix<3, 1> mag_data_;
  BLA::Matrix<3, 1> offset_;
  BLA::Matrix<3, 3> scaling_matrix_;
  BLA::Matrix<3, 3> scaling_matrix_inv_;

private:
  void sphereMapping();
};


