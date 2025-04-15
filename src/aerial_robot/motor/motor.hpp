#pragma once

#include "Arduino.h"
#include <driver/ledc.h>
#include <BasicLinearAlgebra.h>

class Motor
{
public:
  Motor() : motor_pin_(0), motor_channel_(0), pos_({0, 0, 0}), axis_({0, 0, 0}), sigma_(0.0f) {}; // default constructor

  Motor(int motor_pin, int motor_channel,
        BLA::Matrix<3, 1> pos, BLA::Matrix<3, 1> axis,
        float sigma);

  ~Motor() = default;

  void setPwm(float duty);

  void initialize();

  BLA::Matrix<3, 1> getPos() {return pos_;}
  BLA::Matrix<3, 1> getAxis() {return axis_;}
  float getSigma() {return sigma_;}

private:
  int motor_pin_;
  int motor_channel_;
  int motor_resolution_ = 8; // hard coded
  int motor_freq_ = 150000;  // hard coded

  BLA::Matrix<3, 1> pos_;
  BLA::Matrix<3, 1> axis_;
  float sigma_;
};
