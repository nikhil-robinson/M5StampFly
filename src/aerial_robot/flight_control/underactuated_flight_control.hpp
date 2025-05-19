#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>
#include <utils/pid/pid_controller.hpp>
#include <aerial_robot/flight_control/control_base.hpp>
#include <aerial_robot/state_estimation/state_estimator.hpp>
#include <aerial_robot/navigation/navigation_base.hpp>
#include <config.h>

class UnderActuatedFlightController : public ControlBase {
   public:
    UnderActuatedFlightController(std::shared_ptr<Odometry> odom, std::shared_ptr<BaseNavigator> navigator);
    ~UnderActuatedFlightController() = default;

    void update();
    BLA::Matrix<4, 1> getControlInput() {
        return control_input_;
    }
    void dumpControlInput() {
        USBSerial.printf("control input: %f %f %f %f\n", control_input_(0), control_input_(1), control_input_(2),
                         control_input_(3));
    }

   private:
    BLA::Matrix<4, 1> control_input_;
    PIDController x_pid_;
    PIDController y_pid_;
    PIDController z_pid_;
    PIDController roll_pid_;
    PIDController pitch_pid_;
    PIDController yaw_pid_;
};
