#include <Arduino.h>
#include <math.h>
#include "pid.hpp"   // Contains both the PID and Filter classes
#include "flight_control.hpp"
#include "sensor.hpp"
#include "Bitcraze_PMW3901.h"
#include "tof.hpp"
#include "imu.hpp"
#include "button.hpp"
#include "optical_flow.hpp"
#include "pid.hpp"
#include <math.h>

// -------------------- Constants & Parameters --------------------
const float dt = 0.0025;         // Control loop period (400Hz)
const float desiredAltitude = 1.0; // Desired altitude in meters
const float desiredPosX = 0.0;     // Desired horizontal X position (meters)
const float desiredPosY = 0.0;     // Desired horizontal Y position (meters)

// Optical flow scaling: adjust based on your sensor calibration.
// If your sensor outputs velocity instead of displacement, multiply by dt.
const float flowScale = 0.001;     

// Over–G detection threshold (in g's)
const float overGThreshold = 2.0;  

// -------------------- Global State Variables --------------------
// Horizontal position estimates (from integrated optical flow)
float posX = 0.0, posY = 0.0;

// Altitude measurement (from TOF, assumed in meters)
float altitude = 0.0;

// Attitude estimates (in radians) from complementary filtering
float roll  = 0.0;
float pitch = 0.0;

// Over–G fail–safe variables
bool  overG_flag = false;
float overG_value = 0.0;

// -------------------- PID Controllers --------------------
PID altPID;   // Altitude hold (throttle adjustment)
PID posXPID;  // Position hold along X (generates desired pitch correction)
PID posYPID;  // Position hold along Y (generates desired roll correction)

// PID parameters – tune these for your drone
float altKp  = 0.38, altTi  = 10.0, altTd  = 0.5,  altEta = 0.125;
float posKp  = 1.0,  posTi  = 1.0,  posTd  = 0.0,  posEta = 0.01;

// -------------------- Filter Instance --------------------
// This filter is used to smooth the accelerometer norm for over–G detection.
Filter accFilter;


// -------------------- Setup --------------------
void setup() {
    init_copter();

  // Initialize sensors (IMU, TOF, optical flow) here...
  // imu_init();
  // tof_init();
  // optical_flow_init();

  // Initialize PID controllers with the specified gains and dt.
  altPID.set_parameter(altKp, altTi, altTd, altEta, dt);
  posXPID.set_parameter(posKp, posTi, posTd, posEta, dt);
  posYPID.set_parameter(posKp, posTi, posTd, posEta, dt);

  // Initialize the acceleration filter for over–G detection.
  accFilter.set_parameter(0.005, dt);
}

// -------------------- Main Control Loop --------------------
void loop() {
  unsigned long startTime = micros();

  // ---------- Sensor Data Acquisition ----------
  // Read IMU acceleration (in g's) and gyro (in rad/s)
  float acc_x = imu_get_acc_x();
  float acc_y = imu_get_acc_y();
  float acc_z = imu_get_acc_z();
  float gyro_x = imu_get_gyro_x();
  float gyro_y = imu_get_gyro_y();
  // (gyro_z is available if needed—for yaw control—but is not used here)

  // Read altitude from the bottom TOF sensor (assumed in meters)
  altitude = tof_bottom_get_range();

  // Read optical flow data and update horizontal position estimates.
  int16_t flow_dx = 0, flow_dy = 0;
  read_optical_flow(&flow_dx, &flow_dy);
  float dispX = flow_dx * flowScale;  // If sensor outputs velocity, use: flow_dx * flowScale * dt;
  float dispY = flow_dy * flowScale;
  posX += dispX;
  posY += dispY;

  // ---------- Over–G Fail–Safe ----------
  // Compute acceleration norm and filter it to reduce noise.
  float acc_norm = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  float filteredAccNorm = accFilter.update(acc_norm, dt);
  if (filteredAccNorm > overGThreshold) {
      overG_flag = true;
      if (overG_value == 0.0) overG_value = filteredAccNorm;
  } else {
      overG_flag = false;
      overG_value = 0.0;
  }

  // ---------- Attitude Estimation via Complementary Filter ----------
  // Calculate roll and pitch estimates from accelerometer data.
  float accRoll = atan2(acc_y, acc_z);
  float denom = sqrt(acc_y * acc_y + acc_z * acc_z);
  float accPitch = (denom > 0.0001) ? atan2(-acc_x, denom) : pitch;

  // Fuse gyro integration with accelerometer estimates.
  // (Assuming gyro_x and gyro_y are in rad/s.)
  roll  = 0.98 * (roll + gyro_x * dt) + 0.02 * accRoll;
  pitch = 0.98 * (pitch + gyro_y * dt) + 0.02 * accPitch;

  // ---------- Altitude Hold via PID ----------
  float altError = desiredAltitude - altitude;
  float throttleAdjust = altPID.update(altError, dt);
  // Base throttle (e.g., 0.5) adjusted by the altitude PID output.
  float baseThrottle = constrain(0.5 + throttleAdjust, 0.0, 1.0);

  // ---------- Position Hold via PID ----------
  // Compute errors in horizontal (X, Y) positions.
  float errorX = desiredPosX - posX;
  float errorY = desiredPosY - posY;
  // Generate desired tilt corrections (in radians) to bring the drone back
  // to the desired position. Typically, a positive errorX produces a forward tilt.
  float desiredPitch = posXPID.update(errorX, dt);  // Adjusts forward/backward movement
  float desiredRoll  = posYPID.update(errorY, dt);    // Adjusts left/right movement

  // ---------- Motor Mixing ----------
  // The motor duty values are computed by combining the base throttle with tilt corrections.
  // Positive desiredPitch tilts the drone forward; positive desiredRoll tilts it right.
  float duty_fr = baseThrottle - desiredPitch - desiredRoll;
  float duty_fl = baseThrottle - desiredPitch + desiredRoll;
  float duty_rr = baseThrottle + desiredPitch - desiredRoll;
  float duty_rl = baseThrottle + desiredPitch + desiredRoll;

  // Constrain motor commands to the valid range [0, 1]
  duty_fr = constrain(duty_fr, 0.0, 0.9);
  duty_fl = constrain(duty_fl, 0.0, 0.9);
  duty_rr = constrain(duty_rr, 0.0, 0.9);
  duty_rl = constrain(duty_rl, 0.0, 0.9);

  // If an over–G event is detected, shut off motor outputs for safety.
  if (overG_flag) {
    duty_fr = duty_fl = duty_rr = duty_rl = 0.0;
  }

  // Send motor commands
  set_duty_fr(duty_fr);
  set_duty_fl(duty_fl);
  set_duty_rr(duty_rr);
  set_duty_rl(duty_rl);

  USBSerial.printf("%f %f\n",duty_fl,duty_fr);
  USBSerial.printf("%f %f\n",duty_rl,duty_rr);

  // ---------- Loop Timing Control ----------
  // Wait until dt has elapsed (busy–wait). In a production system, consider using timer interrupts.
  while (micros() - startTime < dt * 1000000UL) { }
}
