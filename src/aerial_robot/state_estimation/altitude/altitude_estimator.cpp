#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>

void AltitudeEstimator::initialize() {
    altitude_kalman_filter_.initialize();

    raw_az_filter_.set_parameter(0.003, 0.002);
}

void AltitudeEstimator::update() {
    tof_->update();
    int16_t raw_range = tof_->getTofRange();
    int16_t range     = last_input_tof_;

    if (raw_range > min_tof_) range = raw_range;

    if (std::abs(range - last_input_tof_) > tof_diff_torelance_) range = last_input_tof_;

    float filter_acc_z = raw_az_filter_.update(imu_->getAccZ(), 0.002);

    altitude_kalman_filter_.update((float)range / 1000.0, gravity_ * (filter_acc_z - 1.0));

    last_input_tof_ = range;
}
