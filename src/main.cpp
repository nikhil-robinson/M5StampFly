#include <Arduino.h>
#include <math.h>
#include "pid.hpp"
#include "flight_control.hpp"
#include "sensor.hpp"
#include "Bitcraze_PMW3901.h"
#include "tof.hpp"
#include "imu.hpp"
#include "button.hpp"
#include "optical_flow.hpp"

// ----- Global Variables & Structures -----
// All units are in millimeters (mm) and mm/s for velocity, mm/s^2 for acceleration.
struct State {
  float x;    // horizontal position (mm)
  float y;    // horizontal position (mm)
  float vx;   // horizontal velocity (mm/s)
  float vy;   // horizontal velocity (mm/s)
};

State state = {0, 0, 0, 0};
float totalDistance = 0.0; // cumulative distance in mm

// Kalman filter parameters (placeholders)
// Covariance matrix P (4x4), initially an identity matrix.
float P[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

// Process noise covariance matrix Q (adjust these values based on your system dynamics)
// Units should be consistent with the state (mm and mm/s).
float Q[4][4] = {
  {0.01, 0,    0,    0},
  {0,    0.01, 0,    0},
  {0,    0,    0.1,  0},
  {0,    0,    0,    0.1}
};

// Measurement noise covariance matrix R for the optical flow sensor (2x2)
// Values here represent the variance in mm^2.
float R[2][2] = {
  {0.05, 0},
  {0, 0.05}
};

// Constant for converting optical flow pixels to displacement.
// With altitude in mm and a focal length in pixels, the displacement (mm) is:
// displacement (mm) = pixel displacement * (altitude (mm) / focalLength (pixels))
const float focalLength = 200.0;  // example focal length in pixels

// Timing
unsigned long prevTime = 0;

// ----- Kalman Filter Prediction -----
// Use IMU acceleration (in mm/s^2) for the prediction step.
void kalmanPredict(float ax, float ay, float dt) {
  // Using a constant acceleration model:
  // x_new = x + vx*dt + 0.5*ax*dt^2, vx_new = vx + ax*dt (similarly for y)
  state.x  = state.x  + state.vx * dt + 0.5 * ax * dt * dt;
  state.y  = state.y  + state.vy * dt + 0.5 * ay * dt * dt;
  state.vx = state.vx + ax * dt;
  state.vy = state.vy + ay * dt;
  
  // Simplified covariance update: add process noise along the diagonal.
  P[0][0] += Q[0][0];
  P[1][1] += Q[1][1];
  P[2][2] += Q[2][2];
  P[3][3] += Q[3][3];
}

// ----- Full Covariance Update (Joseph Form) -----
// The full covariance update is given by:
//   P_new = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
// where H is the measurement matrix for optical flow:
//   H = [ [1, 0, 0, 0],
//         [0, 1, 0, 0] ]
void updateCovariance(const float K[4][2]) {
  // Step 1: Compute (I - K·H)
  float I_KH[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      I_KH[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  // Since H only has nonzero entries in the first two columns, subtract K accordingly.
  for (int i = 0; i < 4; i++) {
    I_KH[i][0] -= K[i][0];  // H[0][0] = 1
    I_KH[i][1] -= K[i][1];  // H[1][1] = 1
  }
  
  // Step 2: Compute the intermediate product M = (I - K·H) * P.
  float M[4][4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        M[i][j] += I_KH[i][k] * P[k][j];
      }
    }
  }
  
  // Step 3: Compute P_term = M * (I - K·H)ᵀ.
  float P_term[4][4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        P_term[i][j] += M[i][k] * I_KH[j][k];  // Transposing I_KH by swapping indices
      }
    }
  }
  
  // Step 4: Compute the second term: K * R * Kᵀ.
  float KRKt[4][4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 2; k++) {
        for (int l = 0; l < 2; l++) {
          KRKt[i][j] += K[i][k] * R[k][l] * K[j][l];
        }
      }
    }
  }
  
  // Step 5: Sum the two terms to get the updated covariance matrix.
  float P_new[4][4] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P_new[i][j] = P_term[i][j] + KRKt[i][j];
    }
  }
  
  // Copy the updated covariance matrix back to P.
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P[i][j] = P_new[i][j];
    }
  }
}

// ----- Kalman Filter Update -----
// Update using optical flow displacement measurements, which are in mm.
// disp_x and disp_y are computed from optical flow data.
void kalmanUpdate(float meas_dx, float meas_dy) {
  // Measurement vector z for optical flow (in mm)
  float z[2] = { meas_dx, meas_dy };
  
  // Measurement matrix H (2x4) is:
  //   [1, 0, 0, 0]
  //   [0, 1, 0, 0]
  // The innovation (measurement residual) is: y = z - H*x.
  // For simplicity, if our predicted measurement is zero, then y = z.
  float y_innov[2] = { z[0], z[1] };
  
  // Innovation covariance S = H*P*Hᵀ + R, which is the top-left 2x2 block of P plus R.
  float S[2][2];
  S[0][0] = P[0][0] + R[0][0];
  S[0][1] = P[0][1] + R[0][1];
  S[1][0] = P[1][0] + R[1][0];
  S[1][1] = P[1][1] + R[1][1];
  
  // Compute the inverse of S (2x2 matrix inversion).
  float detS = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  if (detS == 0) return;  // Prevent division by zero.
  float S_inv[2][2];
  S_inv[0][0] = S[1][1] / detS;
  S_inv[0][1] = -S[0][1] / detS;
  S_inv[1][0] = -S[1][0] / detS;
  S_inv[1][1] = S[0][0] / detS;
  
  // Compute the Kalman gain: K = P * Hᵀ * S_inv.
  // Here, Hᵀ is 4x2 (it simply selects the first two columns of P).
  float K[4][2];
  for (int i = 0; i < 4; i++) {
    K[i][0] = P[i][0] * S_inv[0][0] + P[i][1] * S_inv[1][0];
    K[i][1] = P[i][0] * S_inv[0][1] + P[i][1] * S_inv[1][1];
  }
  
  // Update the state: state = state + K * y_innov.
  state.x  = state.x  + K[0][0] * y_innov[0] + K[0][1] * y_innov[1];
  state.y  = state.y  + K[1][0] * y_innov[0] + K[1][1] * y_innov[1];
  state.vx = state.vx + K[2][0] * y_innov[0] + K[2][1] * y_innov[1];
  state.vy = state.vy + K[3][0] * y_innov[0] + K[3][1] * y_innov[1];
  
  // Update the covariance matrix using the Joseph form.
  updateCovariance(K);
}

void setup() {
  // Initialize flight control and sensors.
  init_copter();
  delay(100);
  
  // Initialize sensor objects as needed:
  // opticalFlow.begin();
  // tof.begin();
  // imu.begin();
  
  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // dt in seconds
  if (dt <= 0) return;  // Ensure positive dt
  prevTime = currentTime;
  
  // ----- Sensor Readings -----
  
  // 1. Read optical flow sensor (pixel displacements).
  int16_t pixel_dx = 0, pixel_dy = 0;
  read_optical_flow(&pixel_dx, &pixel_dy);
  USBSerial.printf("Optical Flow Data: X[%d] Y[%d]\n", pixel_dx, pixel_dy);
  
  // 2. Read altitude from TOF sensor (altitude in mm).
  float altitude = tof_bottom_get_range();
  USBSerial.printf("TOF Altitude: %f mm\n", altitude);
  
  // 3. Convert pixel displacement to real-world displacement (mm):
  //    Scale factor = altitude (mm) / focalLength (pixels)
  float disp_x = pixel_dx * (altitude / focalLength);
  float disp_y = pixel_dy * (altitude / focalLength);
  
  // 4. Read IMU data (accelerations in mm/s², gyro values as provided).
  imu_update();
  float ax  = imu_get_acc_x();  // in mm/s²
  float ay  = imu_get_acc_y();  // in mm/s²
  float az  = imu_get_acc_z();  // in mm/s²
  float gx  = imu_get_gyro_x();
  float gy  = imu_get_gyro_y();
  float gz  = imu_get_gyro_z();
  
  USBSerial.printf("Acc Data: X[%f] Y[%f] Z[%f]\n", ax, ay, az);
  USBSerial.printf("Gyro Data: X[%f] Y[%f] Z[%f]\n", gx, gy, gz);
  
  // (Optionally, adjust the optical flow vector using gyro data if the sensor
  // is not aligned with the world frame.)
  
  // ----- Kalman Filter Prediction -----
  kalmanPredict(ax, ay, dt);
  
  // ----- Kalman Filter Update -----
  kalmanUpdate(disp_x, disp_y);
  
  // ----- Distance Calculation -----
  // Integrate changes in state position to compute cumulative horizontal distance (mm).
  static float prev_x = state.x;
  static float prev_y = state.y;
  
  float dx = state.x - prev_x;
  float dy = state.y - prev_y;
  float incrementalDistance = sqrt(dx * dx + dy * dy);
  totalDistance += incrementalDistance;
  
  prev_x = state.x;
  prev_y = state.y;
  
  // ----- Output Results -----
  USBSerial.print("Total Distance (mm): ");
  USBSerial.println(totalDistance);
  
  delay(10);
}
