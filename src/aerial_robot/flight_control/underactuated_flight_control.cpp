#include <aerial_robot/flight_control/underactuated_flight_control.hpp>

UnderActuatedFlightController::UnderActuatedFlightController(std::shared_ptr<Odometry> odom,
                                                             std::shared_ptr<BaseNavigator> navigator)
    : ControlBase(odom, navigator),
      x_pid_(CONTROL::XY_P_GAIN, CONTROL::XY_I_GAIN, CONTROL::XY_D_GAIN),
      y_pid_(CONTROL::XY_P_GAIN, CONTROL::XY_I_GAIN, CONTROL::XY_D_GAIN),
      z_pid_(CONTROL::Z_P_GAIN, CONTROL::Z_I_GAIN, CONTROL::Z_D_GAIN),
      roll_pid_(CONTROL::ROLL_P_GAIN, CONTROL::ROLL_I_GAIN, CONTROL::ROLL_D_GAIN),
      pitch_pid_(CONTROL::PITCH_P_GAIN, CONTROL::PITCH_I_GAIN, CONTROL::PITCH_D_GAIN),
      yaw_pid_(CONTROL::YAW_P_GAIN, CONTROL::YAW_I_GAIN, CONTROL::YAW_D_GAIN) {
    control_input_ = {0.0, 0.0, 0.0, 0.0};
}

void UnderActuatedFlightController::update() {
    int flight_state = navigator_->getFlightState();
    if (flight_state == ARM_OFF_STATE) {
        for (int i = 0; i < 4; i++)  // todo: motor_num
        {
            control_input_(i) = 0;
            x_pid_.reset();
            y_pid_.reset();
            z_pid_.reset();
            roll_pid_.reset();
            pitch_pid_.reset();
            yaw_pid_.reset();
        }
        return;
    }

    BLA::Matrix<3, 1> target_pos = navigator_->getTargetPos();
    BLA::Matrix<3, 1> target_rpy = navigator_->getTargetRpy();

    BLA::Matrix<3, 1> pos = odom_->getPos();
    float altitude        = odom_->getPosZ();
    pos(2)                = altitude;

    BLA::Matrix<3, 1> rpy     = odom_->getRpy();
    BLA::Matrix<3, 1> vel     = odom_->getVel();
    BLA::Matrix<3, 1> ang_vel = odom_->getOmega();

    x_pid_.update(target_pos(0) - pos(0), SYSTEM::MAIN_LOOP_DU / 1000.0, -vel(0));
    y_pid_.update(target_pos(1) - pos(1), SYSTEM::MAIN_LOOP_DU / 1000.0, -vel(1));
    z_pid_.update(target_pos(2) - pos(2), SYSTEM::MAIN_LOOP_DU / 1000.0, -vel(2), CONTROL::Z_OFFSET);

    float cos_psi           = cosf(rpy(2));
    float sin_psi           = sinf(rpy(2));
    BLA::Matrix<3, 3> R_psi = BLA::Eye<3, 3>();
    R_psi(0, 0)             = cos_psi;
    R_psi(0, 1)             = -sin_psi;
    R_psi(1, 0)             = sin_psi;
    R_psi(1, 1)             = cos_psi;

    BLA::Matrix<3, 1> target_acc_cog_dash =
        R_psi * BLA::Matrix<3, 1>(x_pid_.result(), y_pid_.result(), z_pid_.result());

    // calculate target roll, pitch from xyz target acceleration. approximate near hover state
    float target_roll  = -target_acc_cog_dash(1) / ROBOT_MODEL::GRAVITY;
    float target_pitch = target_acc_cog_dash(0) / ROBOT_MODEL::GRAVITY;

    roll_pid_.update(target_roll - rpy(0), SYSTEM::MAIN_LOOP_DU / 1000.0, -ang_vel(0));
    pitch_pid_.update(target_pitch - rpy(1), SYSTEM::MAIN_LOOP_DU / 1000.0, -ang_vel(1));
    yaw_pid_.update(target_rpy(2) - rpy(2), SYSTEM::MAIN_LOOP_DU / 1000.0, -ang_vel(2));

    navigator_->setTargetRoll(target_roll);
    navigator_->setTargetPitch(target_pitch);

    control_input_(0) = z_pid_.result();
    control_input_(1) = roll_pid_.result();
    control_input_(2) = pitch_pid_.result();
    control_input_(3) = yaw_pid_.result();
}
