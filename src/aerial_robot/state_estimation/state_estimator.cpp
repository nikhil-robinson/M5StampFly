#include <aerial_robot/state_estimation/state_estimator.hpp>

Odometry::Odometry(std::shared_ptr<AttitudeEstimator> attitude_estimator,
                   std::shared_ptr<AltitudeEstimator> altitude_estimator):
  attitude_estimator_(attitude_estimator),
  altitude_estimator_(altitude_estimator)
{
  pos_ = {0.0, 0.0, 0.0};
  vel_ = {0.0, 0.0, 0.0};
  acc_ = {0.0, 0.0, 0.0};
  rpy_ = {0.0, 0.0, 0.0};
  omega_ = {0.0, 0.0, 0.0};
  ang_acc_ = {0.0, 0.0, 0.0};
}

void Odometry::update()
{
  // attitude
  attitude_estimator_->update();
  acc_ = attitude_estimator_->getAcc();
  rpy_ = attitude_estimator_->getRpy();
  omega_ = attitude_estimator_->getGyro();

  altitude_estimator_->update();
}


