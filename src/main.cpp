/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <FastLED.h>
#include <vector>
#include <memory>

#include <config.h>


#include <communication/telemetry.hpp>
#include <i2c.hpp>
#include <spi_s3.hpp>

#include <INA3221.h>
#include <opt.hpp>
#include <sensor/imu/imu_bmi270.hpp>
#include <sensor/mag/mag_bmm150.hpp>

#include <aerial_robot/motor/motor.hpp>
#include <aerial_robot/hardware/quadrotor_hardware.hpp>
#include <aerial_robot/flight_control/underactuated_flight_control.hpp>
#include <aerial_robot/wrench_allocation/quadrotor_wrench_allocation.hpp>
#include <aerial_robot/state_estimation/attitude/attitude_estimator.hpp>
#include <aerial_robot/state_estimation/state_estimator.hpp>
#include <aerial_robot/navigation/underactuated_navigation.hpp>

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76

// #define SDA_PIN      (3)
// #define SCL_PIN      (4)

// std::shared_ptr<INA3221> battery_voltage;
std::shared_ptr<Magnetmeter> mag;
std::shared_ptr<Imu> imu;
std::shared_ptr<Odometry> odom;
std::shared_ptr<AttitudeEstimator> attitude_estimator;
std::shared_ptr<AltitudeEstimator> altitude_estimator;

std::shared_ptr<BaseNavigator> navigator;
std::shared_ptr<UnderActuatedFlightController> flight_control;

std::shared_ptr<Motor> motor1;
std::shared_ptr<Motor> motor2;
std::shared_ptr<Motor> motor3;
std::shared_ptr<Motor> motor4;
std::shared_ptr<QuadrotorWrenchAllocation> wrench_allocation;
std::shared_ptr<QuadrotorHardware> hardware;

void setup() {
#if 1
  /* begin debug */
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.printf("serial begin!\r\n");

  // init spi bus
  USBSerial.printf("SPI Initilize status:%d\n\r", spi_init());

  // imu (bmi270)
  imu = std::make_shared<ImuBMI270>();
  imu->initialize();

  // init optical flow
  powerUp(&optconfig);
  initRegisters();

  // i2c
  Wire1.begin(I2C::SDA_PIN, I2C::SCL_PIN, 400000UL);
  // i2c_master_init();
  i2c_scan();

  // battery
  // battery_voltage = std::make_shared<INA3221>(INA3221_ADDR40_GND);
  // battery_voltage->begin(&Wire1);
  // battery_voltage->reset();

  // magnetmeter (bmm150)
  mag = std::make_shared<MagnetmeterBMM150>();
  mag->initialize();

  attitude_estimator = std::make_shared<AttitudeEstimator>(imu, mag);
  altitude_estimator = std::make_shared<AltitudeEstimator>(imu);
  // altitude_estimator->initialize();

  odom = std::make_shared<Odometry>(attitude_estimator,
                                    altitude_estimator);

  navigator = std::make_shared<UnderActuatedNavigator>(odom);
  navigator->initialize();

  flight_control = std::make_shared<UnderActuatedFlightController>(odom, navigator);

  motor1 = std::make_shared<Motor>(MOTOR::pwmFrontRight, MOTOR::FrontRight_motor,
                                   BLA::Matrix<3,1>(ROBOT_MODEL::ROTOR_X, -ROBOT_MODEL::ROTOR_Y, 0.0), BLA::Matrix<3,1>(0, 0, 1.0),
                                   -1 *MOTOR::rotor1_direction * MOTOR::SIGMA); // wrench contribution: --

  motor2 = std::make_shared<Motor>(MOTOR::pwmFrontLeft, MOTOR::FrontLeft_motor,
                                   BLA::Matrix<3,1>(ROBOT_MODEL::ROTOR_X, ROBOT_MODEL::ROTOR_Y, 0.0), BLA::Matrix<3,1>(0, 0, 1.0),
                                   -1 *MOTOR::rotor2_direction * MOTOR::SIGMA); // wrench contribution: +-


  motor3 = std::make_shared<Motor>(MOTOR::pwmRearLeft, MOTOR::RearLeft_motor,
                                   BLA::Matrix<3,1>(-ROBOT_MODEL::ROTOR_X, ROBOT_MODEL::ROTOR_Y, 0.0), BLA::Matrix<3,1>(0, 0, 1.0),
                                   -1 *MOTOR::rotor3_direction * MOTOR::SIGMA); // wrench contribution: ++

  motor4 = std::make_shared<Motor>(MOTOR::pwmRearRight, MOTOR::RearRight_motor,
                                   BLA::Matrix<3,1>(-ROBOT_MODEL::ROTOR_X, -ROBOT_MODEL::ROTOR_Y, 0.0), BLA::Matrix<3,1>(0, 0, 1.0),
                                   -1 * MOTOR::rotor4_direction * MOTOR::SIGMA); // wrench contribution: -+

  std::vector<std::shared_ptr<Motor>> motors(0);
  motors.push_back(motor1);
  motors.push_back(motor2);
  motors.push_back(motor3);
  motors.push_back(motor4);

  wrench_allocation = std::make_shared<QuadrotorWrenchAllocation>(motors);

  hardware = std::make_shared<QuadrotorHardware>(motors);

  delay(100);
  /* end debug*/
#endif

#if 0
  /* begin deploy */
  init_copter();
  delay(100);
  /* end deploy */
#endif
}

void loop() {
#if 1
  /* begin debug */
  // optical flow (PMW3901)
  // int16_t flow_deltaX, flow_deltaY;
  // readMotionCount(&flow_deltaX, &flow_deltaY);
  // USBSerial.print("X: ");
  // USBSerial.print(flow_deltaX);
  // USBSerial.print(", Y: ");
  // USBSerial.print(flow_deltaY);
  // USBSerial.print("\n");

  // attitude_estimator->update();
  // altitude_estimator->update();

  navigator->update();
  // flight_control->update();

  // wrench_allocation->update(flight_control->getControlInput());
  // hardware->update(wrench_allocation->getActuatorInput());

  // imu (bmi270)
  // imu_update();
  // float gyro_x = imu_get_gyro_x();
  // float gyro_y = imu_get_gyro_y();
  // float gyro_z = imu_get_gyro_z();
  // float acc_x = imu_get_acc_x();
  // float acc_y = imu_get_acc_y();

  // float acc_z = imu_get_acc_z();
  // imu->update();
  // USBSerial.print("acc: ");
  // USBSerial.print(imu->getAcc());
  // USBSerial.print("gyro: ");
  // USBSerial.print(imu->getGyro());
  // USBSerial.print("\n\n");

  // USBSerial.printf("gyro: %8.4f %8.4f %8.4f\n", gyro_x, gyro_y, gyro_z);
  // USBSerial.printf("acc: %8.4f %8.4f %8.4f\n", acc_x, acc_y, acc_z);

  // USBSerial.printf("Voltage: %f\n\r", battery_voltage->getVoltage(INA3221_CH2));

  // mag->update();
  // USBSerial.print("mag: ");
  // BLA::Matrix<3,1> raw_mag_data = mag->getRawMag();
  // USBSerial.print(raw_mag_data(0));
  // USBSerial.print(" ");
  // USBSerial.print(raw_mag_data(1));
  // USBSerial.print(" ");
  // USBSerial.print(raw_mag_data(2));
  // USBSerial.print("\n");
  // USBSerial.print(magnetmeter->getMag());
  // USBSerial.print("\n\n");

  // USBSerial.print("\n");
  // double T,P;
  // uint8_t result = pressure_.startMeasurment();
  // USBSerial.print("delay: ");
  // USBSerial.print(result);
  // USBSerial.print(" ");
  // delay(result);
  // result = pressure_.getTemperatureAndPressure(T,P);

  // USBSerial.print(T);
  // USBSerial.print(" ");
  // USBSerial.print(P);
  // USBSerial.println();
  telemetry();


  delay(10);
  /* end debug */
#endif

#if 0
  /* begin deploy */
  loop_400Hz();
  /* end deploy */
#endif

}
