#include <aerial_robot/state_estimation/state_estimator.hpp>

Odometry::Odometry(std::shared_ptr<AttitudeEstimator> attitude_estimator,
                   std::shared_ptr<AltitudeEstimator> altitude_estimator,
                   std::shared_ptr<PositionEstimator> position_estimator)
    : attitude_estimator_(attitude_estimator),
      altitude_estimator_(altitude_estimator),
      position_estimator_(position_estimator) {
}

void Odometry::update() {
    // attitude
    attitude_estimator_->update();

    // altitude
    // altitude_estimator_->update();

    // position
    // position_estimator_->update();
}
