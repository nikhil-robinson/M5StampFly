
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

// Pseudocode for Arduino sensor fusion with Kalman filter

// ----- Global Variables & Structures -----
struct State {
  float x;    // horizontal position x (meters)
  float y;    // horizontal position y (meters)
  float vx;   // horizontal velocity in x (m/s)
  float vy;   // horizontal velocity in y (m/s)
};

State state = {0, 0, 0, 0};
float totalDistance = 0.0;

// Kalman filter parameters (for simplicity, these are placeholders)
float P[4][4];         // Error covariance matrix
float Q[4][4];         // Process noise covariance matrix
float R[2][2];         // Measurement noise covariance matrix

// Assume these are declared elsewhere as sensor objects
// OpticalFlow opticalFlow;
// TOFSensor tof;
// Accelerometer accel;
// Gyroscope gyro;

// Constant for converting optical flow pixels to angle (depends on sensor characteristics)
const float focalLength = 200.0;  // example value (in pixels)

// Timing
unsigned long prevTime = 0;

// ----- Kalman Filter Functions -----
void kalmanPredict(float ax, float ay, float dt) {
  // Predict new state using simple kinematics:
  // x_new = x + vx*dt + 0.5*ax*dt^2, and vx_new = vx + ax*dt (similar for y)
  state.x  = state.x  + state.vx * dt + 0.5 * ax * dt * dt;
  state.y  = state.y  + state.vy * dt + 0.5 * ay * dt * dt;
  state.vx = state.vx + ax * dt;
  state.vy = state.vy + ay * dt;
  
  // Update the error covariance P (this is a placeholder update)
  // In practice, you would update P using the state transition model and Q.
}

void kalmanUpdate(float meas_dx, float meas_dy) {
  // Measurement vector: optical flow displacement in x and y (meters)
  float z[2] = { meas_dx, meas_dy };

  // Measurement model: assume we directly measure the position increments
  // Innovation: (z - H*x) where H is the measurement matrix.
  // For simplicity, assume H is identity for position increments.
  float y_innov[2];
  y_innov[0] = z[0] - 0; // difference between measured displacement and predicted displacement (assumed 0 here)
  y_innov[1] = z[1] - 0;
  
  // Compute Kalman gain K (this is simplified; in practice, use full matrix math)
  float K[4][2]; // Kalman gain matrix (state dimension x measurement dimension)
  // ... compute K based on P, H, and R ...
  
  // Update state estimate: state = state + K * innovation
  // For pseudocode, we'll assume a simple proportional correction:
  state.x  = state.x  + K[0][0] * y_innov[0] + K[0][1] * y_innov[1];
  state.y  = state.y  + K[1][0] * y_innov[0] + K[1][1] * y_innov[1];
  state.vx = state.vx + K[2][0] * y_innov[0] + K[2][1] * y_innov[1];
  state.vy = state.vy + K[3][0] * y_innov[0] + K[3][1] * y_innov[1];

  // Update covariance matrix P accordingly.
}

void setup() {
  init_copter();
  delay(100);

  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // time difference in seconds
  if (dt <= 0) return;  // ensure dt is positive
  prevTime = currentTime;
  
  // ----- Sensor Readings -----
  
  // 1. Read optical flow sensor (pixel displacements)
  int16_t pixel_dx = 0, pixel_dy = 0;
  read_optical_flow(&pixel_dx,&pixel_dy);
  
  // 2. Read altitude from TOF sensor (in meters)
  float altitude = tof_bottom_get_range();
  
  // 3. Convert pixel displacement to real-world displacement:
  //    Scale factor: distance (altitude) divided by focal length.
  float disp_x = pixel_dx * (altitude / focalLength);
  float disp_y = pixel_dy * (altitude / focalLength);
  
  // 4. Read accelerometer data (linear acceleration in m/s^2)
  float ax = 0, ay = 0, az = 0;
  // accel.read(&ax, &ay, &az);
  
  // 5. Read gyroscope data (angular rates in deg/s or rad/s)
  float gx = 0, gy = 0, gz = 0;
  // gyro.read(&gx, &gy, &gz);

  imu_update();
  ax  = imu_get_acc_x();
  ay  = imu_get_acc_y();
  az  = imu_get_acc_z();
  gx =  imu_get_gyro_x();
  gy =  imu_get_gyro_y();
  gz =  imu_get_gyro_z();
  
  // Optionally: use gyro data to adjust or rotate optical flow vector if sensor is not aligned with world frame.
  
  // ----- Kalman Filter Prediction -----
  // Use accelerometer data (after any necessary filtering and coordinate transforms)
  kalmanPredict(ax, ay, dt);
  
  // ----- Kalman Filter Update -----
  // Update the filter with the displacement measurement from the optical flow sensor
  kalmanUpdate(disp_x, disp_y);
  
  // ----- Distance Calculation -----
  // Here, we integrate the position estimate changes to compute cumulative distance.
  static float prev_x = state.x;
  static float prev_y = state.y;
  
  float dx = state.x - prev_x;
  float dy = state.y - prev_y;
  float incrementalDistance = sqrt(dx * dx + dy * dy);
  totalDistance += incrementalDistance;
  
  // Update previous state positions
  prev_x = state.x;
  prev_y = state.y;
  
  // ----- Output Results -----
  USBSerial.print("Total Distance (m): ");
  USBSerial.println(totalDistance);
  
  // Add an appropriate delay for your sensor update rate
  delay(10);
}
