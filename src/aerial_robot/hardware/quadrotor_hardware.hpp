#pragma once

#include <vector>
#include <aerial_robot/motor/motor.hpp>
#include <memory>

class QuadrotorHardware {
   public:
    QuadrotorHardware(std::vector<std::shared_ptr<Motor>> motors);
    ~QuadrotorHardware() = default;

    void update(BLA::Matrix<4, 1> actuator_input);
    void dumpMotorPwm() {
        USBSerial.printf("motor pwm: %f %f %f %f\n", motor_pwms_(0), motor_pwms_(1), motor_pwms_(2), motor_pwms_(3));
    }
    void setBatteryVoltage(float voltage) {
        battery_voltage_ = voltage;
    }

   private:
    float clamp(float value, float min_val, float max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    std::vector<std::shared_ptr<Motor>> motors_;
    BLA::Matrix<4, 1> motor_pwms_ = {0.0, 0.0, 0.0, 0.0};

    float battery_voltage_ = 0.0;
};
