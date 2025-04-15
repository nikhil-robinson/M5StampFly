#pragma once

#include <sensor/imu/imu.hpp>
#include <aerial_robot/state_estimation/altitude/alt_kalman.hpp>
#include <memory>
#include <utils/lpf/lpf.hpp>
#include "tof.hpp"

class AltitudeEstimator
{
public:
  AltitudeEstimator(std::shared_ptr<Imu> imu):
    imu_(imu)
  {}
  ~AltitudeEstimator() = default;

  void initialize();
  void update();

private:
  std::shared_ptr<Imu> imu_;
  Alt_kalman altitude_kalman_filter_;
  uint8_t ToF_bottom_data_ready_flag_ = 0;

  Filter raw_az_filter_;
  Filter raw_az_d_filter_;
  Filter az_filter_;
};
