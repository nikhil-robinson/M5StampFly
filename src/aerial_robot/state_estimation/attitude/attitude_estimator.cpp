#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>

AttitudeEstimator::AttitudeEstimator(std::shared_ptr<Imu> imu, std::shared_ptr<Magnetmeter> magnetmeter):
  imu_(imu),
  magnetmeter_(magnetmeter)
{
  imu_last_update_time_ = 0.0;
  mag_last_update_time_ = 0.0;
}

void AttitudeEstimator::update()
{
  if(imu_last_update_time_ > IMU_UPDATE_DU)
    {
      imu_->update();
      imu_last_update_time_ = 0.0;
    }

  if(mag_last_update_time_ > MAG_UPDATE_DU)
    {
      magnetmeter_->update();
      mag_last_update_time_ = 0.0;
    }

  complementary_filter_.setAcc(imu_->getAcc());
  complementary_filter_.setGyro(imu_->getGyro());
  complementary_filter_.setMag(magnetmeter_->getMag());

  complementary_filter_.estimation();

  // USBSerial.print("gyro: ");
  // USBSerial.print(complementary_filter_.getGyro());
  // USBSerial.print("\n");
  // USBSerial.print("acc: ");
  // USBSerial.print(complementary_filter_.getAcc());
  // USBSerial.print("\n");
  // USBSerial.print("mag: ");
  // USBSerial.print(complementary_filter_.getMag());
  // USBSerial.print("\n");
  // USBSerial.print("rpy: ");
  // USBSerial.print(complementary_filter_.getRpy());
  // USBSerial.print("\n");
  // USBSerial.print("est_m: ");
  // USBSerial.print(complementary_filter_.getEstM());
  // USBSerial.print("\n");
  // USBSerial.print("est_g: ");
  // USBSerial.print(complementary_filter_.getEstG());
  // USBSerial.print("\n");
  // USBSerial.print("\n");
  // USBSerial.print("\n");
  // USBSerial.print("\n");

  imu_last_update_time_ += DELTA_T;
  mag_last_update_time_ += DELTA_T;
}
