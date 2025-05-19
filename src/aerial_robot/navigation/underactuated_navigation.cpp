#include <aerial_robot/navigation/underactuated_navigation.hpp>

void UnderActuatedNavigator::update() {
    if ((flight_state_ != ARM_OFF_STATE) && option_button) {
        flight_state_ = ARM_OFF_STATE;
        USBSerial.print("emergency stop\n");
        return;
    }

    // mode change
    switch (flight_state_) {
        case ARM_OFF_STATE: {
            if (Stick[BUTTON_ARM] && Stick[BUTTON_FLIP]) {
                flight_state_  = TAKEOFF_STATE;
                target_pos_(0) = odom_->getPos()(0);
                target_pos_(1) = odom_->getPos()(1);
                target_pos_(2) = NAVIGATION::TAKEOFF_HEIGHT;
                target_rpy_(2) = odom_->getYaw();
                USBSerial.print("change to takeoff state\n");
            }
            break;
        }
        case TAKEOFF_STATE: {
            if (Stick[CONTROLMODE] && Stick[ALTCONTROLMODE]) {
                flight_state_  = LAND_STATE;
                target_pos_(0) = odom_->getPos()(0);
                target_pos_(1) = odom_->getPos()(1);
                target_pos_(2) = NAVIGATION::LANDING_HEIGHT;
                target_rpy_(2) = odom_->getYaw();
                USBSerial.print("change to landing state\n");
            }
            break;
        }
        default:
            break;
    }

    // navigation for each state
    switch (flight_state_) {
        case LAND_STATE: {
            if (std::abs(odom_->getPosZ() - NAVIGATION::LANDING_HEIGHT) < NAVIGATION::ARM_OFF_THRESHOLD) {
                flight_state_ = ARM_OFF_STATE;
                USBSerial.print("change to arm off state\n");
            }
            break;
        }
        default:
            break;
    }
}
