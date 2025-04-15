#pragma once

#include <sensor/imu/imu.hpp>
#include <sensor/mag/mag.hpp>
#include <memory>
#include <BasicLinearAlgebra.h>
#include <aerial_robot/state_estimation/attitude/complementary_ahrs.hpp>

#define DELTA_T 0.01f
#define IMU_UPDATE_HZ 400.0f
#define IMU_UPDATE_DU (1.0f / IMU_UPDATE_HZ)
#define MAG_UPDATE_HZ 30.0f
#define MAG_UPDATE_DU (1.0f / MAG_UPDATE_HZ)
class AttitudeEstimator
{
public:
  AttitudeEstimator(std::shared_ptr<Imu> imu, std::shared_ptr<Magnetmeter> magnetmeter);
  ~AttitudeEstimator() = default;

  void update();

  std::shared_ptr<Imu> imu_;
  std::shared_ptr<Magnetmeter> magnetmeter_;
  ComplementaryAHRS complementary_filter_;

  float imu_last_update_time_;
  float mag_last_update_time_;

  BLA::Matrix<3, 1> getAcc() {return complementary_filter_.getAcc();}
  BLA::Matrix<3, 1> getGyro() {return complementary_filter_.getGyro();}
  BLA::Matrix<3, 1> getRpy() {return complementary_filter_.getRpy();}

private:
  float invSqrt(float x);
};
