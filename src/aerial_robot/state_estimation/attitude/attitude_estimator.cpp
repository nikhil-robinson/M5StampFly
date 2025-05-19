#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>

AttitudeEstimator::AttitudeEstimator(std::shared_ptr<Imu> imu, std::shared_ptr<Magnetmeter> magnetmeter)
    : imu_(imu), magnetmeter_(magnetmeter) {
    acc_x_lpf_.set_parameter(0.003, 0.002);
    acc_y_lpf_.set_parameter(0.003, 0.002);
    acc_z_lpf_.set_parameter(0.003, 0.002);
    filtered_acc_ = BLA::Matrix<3, 1>(0.0, 0.0, 0.0);
}

void AttitudeEstimator::update() {
    imu_->update();
    magnetmeter_->update();

    filtered_acc_(0) = acc_x_lpf_.update(imu_->getAccX(), 0.002);
    filtered_acc_(1) = acc_y_lpf_.update(imu_->getAccY(), 0.002);
    filtered_acc_(2) = acc_z_lpf_.update(imu_->getAccZ(), 0.002);

    complementary_filter_.setAcc(imu_->getAcc());
    complementary_filter_.setGyro(imu_->getGyro());
    complementary_filter_.setMag(magnetmeter_->getMag());

    complementary_filter_.estimation();
}

BLA::Matrix<3, 3> AttitudeEstimator::getRotationMatrix() {
    BLA::Matrix<3, 3> rotation_matrix;
    float roll  = complementary_filter_.getRpy()(0);
    float pitch = complementary_filter_.getRpy()(1);
    float yaw   = complementary_filter_.getRpy()(2);

    float c_roll  = cosf(roll);
    float s_roll  = sinf(roll);
    float c_pitch = cosf(pitch);
    float s_pitch = sinf(pitch);
    float c_yaw   = cosf(yaw);
    float s_yaw   = sinf(yaw);

    rotation_matrix(0, 0) = c_pitch * c_yaw;
    rotation_matrix(0, 1) = c_yaw * s_roll * s_pitch - c_roll * s_yaw;
    rotation_matrix(0, 2) = c_roll * c_yaw * s_pitch + s_roll * s_yaw;
    rotation_matrix(1, 0) = c_pitch * s_yaw;
    rotation_matrix(1, 1) = c_roll * c_yaw + s_roll * s_pitch * s_yaw;
    rotation_matrix(1, 2) = -c_yaw * s_roll + c_roll * s_pitch * s_yaw;
    rotation_matrix(2, 0) = -s_pitch;
    rotation_matrix(2, 1) = c_pitch * s_roll;
    rotation_matrix(2, 2) = c_roll * c_pitch;

    return rotation_matrix;
}
