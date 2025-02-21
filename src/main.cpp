#include <Arduino.h>
#include <FastLED.h>
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
// Control loop sample period (400 Hz = 2.5 ms)
const float dt = 0.0025; 

// Conversion factors and filter parameters
const float deg2rad = 0.0174533;
const float alpha   = 0.98;    // Complementary filter weight for attitude

// Desired hover state (tune as needed)
const float desiredAltitude = 1.0;   // in meters
const float desiredPosX     = 0.0;   // horizontal X position (m)
const float desiredPosY     = 0.0;   // horizontal Y position (m)

// Optical flow conversion factor (units->meters)
// This factor converts the raw optical flow units into meters of displacement.
const float flowScale = 0.001; // adjust based on your sensor calibration

// -------------------- Global State Variables --------------------
// Horizontal state estimate (from optical flow)
float posX = 0.0, posY = 0.0;

// Attitude (angles estimated using a complementary filter)
float roll  = 0.0;   // around X-axis
float pitch = 0.0;   // around Y-axis

// -------------------- PID Controllers --------------------
PID altitudePID; // Controls overall throttle for altitude
PID pitchPID;    // Generates desired pitch angle to correct X drift
PID rollPID;     // Generates desired roll angle to correct Y drift

// PID parameters (tune these for your drone)
float altitudeKp = 1.0, altitudeTi = 1.0, altitudeTd = 0.0, altitudeEta = 0.01;
float pitchKp    = 1.0, pitchTi    = 1.0, pitchTd    = 0.0, pitchEta    = 0.01;
float rollKp     = 1.0, rollTi     = 1.0, rollTd     = 0.0, rollEta     = 0.01;

// -------------------- Sensor Functions (provided externally) --------------------
// These functions are assumed to be defined elsewhere (or in your sensor libraries):
//   - float imu_get_acc_x(), imu_get_acc_y(), imu_get_acc_z();
//   - float imu_get_gyro_x(), imu_get_gyro_y(), imu_get_gyro_z();
//   - float tof_bottom_get_range();   // Returns altitude (in meters or units you convert)
//   - void read_optical_flow(int16_t *dx, int16_t *dy);
//   - void set_duty_fr(float duty), set_duty_fl(float duty),
//         set_duty_rr(float duty), set_duty_rl(float duty);

// -------------------- Setup --------------------


#if 1
void setup() {
    init_copter();
    altitudePID.set_parameter(altitudeKp, altitudeTi, altitudeTd, altitudeEta, dt);
    pitchPID.set_parameter(pitchKp, pitchTi, pitchTd, pitchEta, dt);
    rollPID.set_parameter(rollKp, rollTi, rollTd, rollEta, dt);
    delay(100);
}

void loop() {
        while (Loop_flag == 0);
        Loop_flag = 0;
      
        // ---------- Read Sensor Data ----------
        // Read IMU accelerations and gyros (assumed calibrated and in proper units)
        float acc_x = imu_get_acc_x();
        float acc_y = imu_get_acc_y();
        float acc_z = imu_get_acc_z();
        float gyro_x = imu_get_gyro_x();
        float gyro_y = imu_get_gyro_y();
        float gyro_z = imu_get_gyro_z();
      
        // Read altitude from the bottom TOF sensor
        float rawRange = tof_bottom_get_range();
        // For this example, assume rawRange is in meters. Otherwise, convert as needed.
        float altitude = rawRange;
      
        // Read optical flow to estimate horizontal displacement
        int16_t flow_dx = 0, flow_dy = 0;
        read_optical_flow(&flow_dx, &flow_dy);
        // Convert optical flow reading to displacement (in meters)
        float dispX = flow_dx * flowScale;
        float dispY = flow_dy * flowScale;
        // Update horizontal position estimate
        posX += dispX;
        posY += dispY;
      
        // ---------- Attitude Estimation using Complementary Filter ----------
        // Estimate roll and pitch from accelerometer (in radians)
        // These equations assume that the sensor is mounted such that:
        //   - Roll: rotation around X-axis, and
        //   - Pitch: rotation around Y-axis.
        float accRoll  = atan2(acc_y, acc_z);
        float accPitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
        // Complementary filter combines gyro integration and accelerometer estimation.
        roll  = alpha * (roll + gyro_x * dt) + (1.0 - alpha) * accRoll;
        pitch = alpha * (pitch + gyro_y * dt) + (1.0 - alpha) * accPitch;
      
        // ---------- PID Control Calculations ----------
        // Altitude control: compute error and update PID to adjust throttle
        float altitudeError = desiredAltitude - altitude;
        float throttleAdjust = altitudePID.update(altitudeError, dt);
        // Base throttle: choose a hover value (e.g., 0.5) and add the PID output.
        float baseThrottle = constrain(0.5 + throttleAdjust, 0.0, 1.0);
      
        // Horizontal control: compute position error in X and Y
        float posErrorX = desiredPosX - posX;
        float posErrorY = desiredPosY - posY;
        // Use PID to generate a desired tilt angle (in radians) to correct drift.
        float desiredPitch = pitchPID.update(posErrorX, dt); // forward/back tilt
        float desiredRoll  = rollPID.update(posErrorY, dt);    // left/right tilt
      
        // ---------- Motor Mixing ----------
        // To correct horizontal position, we tilt the drone:
        //   - A forward/back tilt (pitch) adjusts the drone in the X direction.
        //   - A left/right tilt (roll) adjusts the drone in the Y direction.
        // Motor adjustments are mixed as follows:
        float FrontRight_motor_duty = baseThrottle - desiredPitch - desiredRoll;
        float FrontLeft_motor_duty  = baseThrottle - desiredPitch + desiredRoll;
        float RearRight_motor_duty  = baseThrottle + desiredPitch - desiredRoll;
        float RearLeft_motor_duty   = baseThrottle + desiredPitch + desiredRoll;
      
        // Constrain motor commands to valid range [0, 1]
        FrontRight_motor_duty = constrain(FrontRight_motor_duty, 0.0, 1.0);
        FrontLeft_motor_duty  = constrain(FrontLeft_motor_duty,  0.0, 1.0);
        RearRight_motor_duty  = constrain(RearRight_motor_duty,  0.0, 1.0);
        RearLeft_motor_duty   = constrain(RearLeft_motor_duty,   0.0, 1.0);
      
        // Set motor outputs using your provided functions
        set_duty_fr(FrontRight_motor_duty);
        set_duty_fl(FrontLeft_motor_duty);
        set_duty_rr(RearRight_motor_duty);
        set_duty_rl(RearLeft_motor_duty);

        USBSerial.printf("%f %f %f %f\r\n",FrontRight_motor_duty,FrontLeft_motor_duty,RearRight_motor_duty,RearLeft_motor_duty);


}


#else

Bitcraze_PMW3901 flow(12);


void setup() {
    // delay(1000 * 10);
    // init_copter();
    init_button();
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly!\r\n");

    sensor_init();
    while (!flow.begin()) {
        USBSerial.printf("Initialization of the flow sensor failed\r\n");
    }

    USBSerial.printf("Finish sensor init!\r\n");
    delay(100);
}

void loop() 
{
    int16_t deltaX,deltaY;
    flow.readMotionCount(&deltaX, &deltaY);
    imu_update();  // IMUの値を読む前に必ず実行
    float acc_x  = imu_get_acc_x();
    float acc_y  = imu_get_acc_y();
    float acc_z  = imu_get_acc_z();
    float gyro_x = imu_get_gyro_x();
    float gyro_y = imu_get_gyro_y();
    float gyro_z = imu_get_gyro_z();

    USBSerial.printf("X[%d] Y[%d]\r\n",deltaX,deltaY);
    USBSerial.printf("%d mm\r\n",tof_bottom_get_range());
    USBSerial.printf("ACC-X[%f] ACC-Y[%f] ACC-Z[%f]\r\n",acc_x,acc_y,acc_z);
    USBSerial.printf("GYRO-X[%f] GYRO-Y[%f] GYRO-Z[%f]\r\n",gyro_x,gyro_y,gyro_z);
    delay(100);

}

#endif