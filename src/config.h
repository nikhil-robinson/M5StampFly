#pragma once

namespace I2C {
constexpr int SDA_PIN = 3;
constexpr int SCL_PIN = 4;
}  // namespace I2C

namespace SYSTEM {
constexpr float MAIN_LOOP_RATE  = 500.0f;
constexpr int MAIN_LOOP_DU      = 2;
constexpr int TELEMETRY_TASK_DU = 200;
constexpr int LED_TASK_DU       = 1000;
}  // namespace SYSTEM

namespace SENSOR {
constexpr float IMU_UPDATE_HZ          = 500.0f;
constexpr float IMU_UPDATE_DU          = 1.0f / IMU_UPDATE_HZ;
constexpr float MAG_UPDATE_HZ          = 30.0f;
constexpr float MAG_UPDATE_DU          = 1.0f / MAG_UPDATE_HZ;
constexpr float OPTICAL_FLOW_UPDATE_HZ = 30.0f;
constexpr float OPTICAL_FLOW_UPDATE_DU = 1.0f / OPTICAL_FLOW_UPDATE_HZ;
constexpr float TOF_UPDATE_HZ          = 30.0f;
constexpr float TOF_UPDATE_DU          = 1.0f / TOF_UPDATE_HZ;
}  // namespace SENSOR

namespace ROBOT_MODEL {
constexpr float GRAVITY = 9.80665f;
constexpr float ROTOR_X = 0.03;  // todo
constexpr float ROTOR_Y = 0.03;  // todo
}  // namespace ROBOT_MODEL

namespace NAVIGATION {
constexpr float TAKEOFF_HEIGHT    = 0.8;
constexpr float LANDING_HEIGHT    = 0.05;
constexpr float ARM_OFF_THRESHOLD = 0.05;
}  // namespace NAVIGATION

namespace MOTOR {
constexpr int pwmFrontLeft     = 5;
constexpr int pwmFrontRight    = 42;
constexpr int pwmRearLeft      = 10;
constexpr int pwmRearRight     = 41;
constexpr int FrontLeft_motor  = 0;
constexpr int FrontRight_motor = 1;
constexpr int RearLeft_motor   = 2;
constexpr int RearRight_motor  = 3;
constexpr int freq             = 150000;
constexpr int resolution       = 8;

constexpr float SIGMA            = 0.05;  // todo
constexpr float rotor1_direction = -1;
constexpr float rotor2_direction = 1;
constexpr float rotor3_direction = -1;
constexpr float rotor4_direction = 1;

constexpr float MIN_PWM_RATIO          = 0.0;
constexpr float MIN_ROTATING_PWM_RATIO = 0.1;  // todo
constexpr float MAX_PWM_RATIO          = 0.95;
}  // namespace MOTOR

namespace CONTROL {
constexpr float XY_P_GAIN       = 0.0;
constexpr float XY_I_GAIN       = 0.0;
constexpr float XY_D_GAIN       = 0.0;
constexpr float XY_OUTPUT_LIMIT = 10.0;
constexpr float XY_I_LIMIT      = 10.0;
constexpr float XY_ROLL_GAIN    = 1.0;
constexpr float XY_PITCH_GAIN   = 1.0;

constexpr float Z_P_GAIN       = 0.0;
constexpr float Z_I_GAIN       = 0.0;
constexpr float Z_D_GAIN       = 0.0;
constexpr float Z_OUTPUT_LIMIT = 50.0;
constexpr float Z_I_LIMIT      = 50.0;
constexpr float Z_OFFSET       = 5.0;

constexpr float ROLL_P_GAIN       = 0.1;
constexpr float ROLL_I_GAIN       = 0.0;
constexpr float ROLL_D_GAIN       = 0.1;
constexpr float ROLL_OUTPUT_LIMIT = 4.0;
constexpr float ROLL_I_LIMIT      = 4.0;

constexpr float PITCH_P_GAIN       = 0.1;
constexpr float PITCH_I_GAIN       = 0.0;
constexpr float PITCH_D_GAIN       = 0.1;
constexpr float PITCH_OUTPUT_LIMIT = 4.0;
constexpr float PITCH_I_LIMIT      = 4.0;

constexpr float YAW_P_GAIN       = 0.0;
constexpr float YAW_I_GAIN       = 0.0;
constexpr float YAW_D_GAIN       = 0.0;
constexpr float YAW_OUTPUT_LIMIT = 10.0;
constexpr float YAW_I_LIMIT      = 5.0;
}  // namespace CONTROL
