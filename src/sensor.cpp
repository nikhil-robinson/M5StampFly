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

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "opt.hpp"
#include "flight_control.hpp"
#include <math.h>

Madgwick Drone_ahrs;
Alt_kalman EstimatedAltitude;

INA3221 ina3221(INA3221_ADDR40_GND);
Filter acc_filter;
Filter az_filter;
Filter voltage_filter;
Filter raw_ax_filter;
Filter raw_ay_filter;
Filter raw_az_filter;
Filter raw_az_d_filter;
Filter raw_gx_filter;
Filter raw_gy_filter;
Filter raw_gz_filter;
Filter alt_filter;
Filter x_vel_filter;
Filter y_vel_filter;

// Sensor data
volatile float Roll_angle = 0.0f, Pitch_angle = 0.0f, Yaw_angle = 0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Roll_rate_offset = 0.0f, Pitch_rate_offset = 0.0f, Yaw_rate_offset = 0.0f;
volatile float Accel_z_d;
volatile float Accel_z_offset = 0.0f;
volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
volatile float Accel_x, Accel_y, Accel_z;
volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
volatile float Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
volatile int16_t RawRange      = 0;
volatile int16_t Range         = 0;
volatile int16_t RawRangeFront = 0;
volatile int16_t RangeFront    = 0;
volatile float Altitude        = 0.0f;
volatile float Altitude2       = 0.0f;
volatile float Alt_velocity    = 0.0f;
volatile float Az              = 0.0;
volatile float Az_bias         = 0.0;
int16_t deltaX, deltaY;

volatile uint16_t Offset_counter = 0;

volatile float Voltage;
float Acc_norm = 0.0f;
float Over_g = 0.0f, Over_rate = 0.0f;
uint8_t OverG_flag                  = 0;
uint8_t Range0flag                  = 0;
volatile uint8_t Under_voltage_flag = 0;

uint8_t scan_i2c() {
    USBSerial.println("I2C scanner. Scanning ...");
    delay(50);
    byte count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        Wire1.beginTransmission(i);
        if (Wire1.endTransmission() == 0) {
            USBSerial.print("Found address: ");
            USBSerial.print(i, DEC);
            USBSerial.print(" (0x");
            USBSerial.print(i, HEX);
            USBSerial.println(")");
            count++;
        }
    }
    USBSerial.print("Found ");
    USBSerial.print(count, DEC);
    USBSerial.println(" device(s).");
    return count;
}

void sensor_reset_offset(void) {
    Roll_rate_offset  = 0.0f;
    Pitch_rate_offset = 0.0f;
    Yaw_rate_offset   = 0.0f;
    Accel_z_offset    = 0.0f;
    Offset_counter    = 0;
}

void sensor_calc_offset_avarage(void) {
    Roll_rate_offset  = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
    Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
    Yaw_rate_offset   = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
    Accel_z_offset    = (Offset_counter * Accel_z_offset + Accel_z_raw) / (Offset_counter + 1);
    Offset_counter++;
}

void test_voltage(void) {
    for (uint16_t i = 0; i < 1000; i++) {
        USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
    }
}

void ahrs_reset(void) {
    Drone_ahrs.reset();
}

void sensor_init() {
    Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);
    if (scan_i2c() == 0) {
        USBSerial.printf("No I2C device!\r\n");
        USBSerial.printf("Can not boot AtomFly2.\r\n");
        while (1);
    }
    USBSerial.printf("SPI Initilize status:%d\n\r", spi_init());
    tof_init();
    imu_init();
    opt_init();
    Drone_ahrs.begin(400.0);
    ina3221.begin(&Wire1);
    ina3221.reset();
    voltage_filter.set_parameter(0.005, 0.0025);
    uint16_t cnt = 0;
    while (cnt < 10) {
        if (ToF_bottom_data_ready_flag) {
            ToF_bottom_data_ready_flag = 0;
            cnt++;
            USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
        }
    }
    delay(10);
    acc_filter.set_parameter(0.005, 0.0025);
    raw_ax_filter.set_parameter(0.003, 0.0025);
    raw_ay_filter.set_parameter(0.003, 0.0025);
    raw_az_filter.set_parameter(0.003, 0.0025);
    raw_gx_filter.set_parameter(0.003, 0.0025);
    raw_gy_filter.set_parameter(0.003, 0.0025);
    raw_gz_filter.set_parameter(0.003, 0.0025);
    raw_az_d_filter.set_parameter(0.1, 0.0025);
    az_filter.set_parameter(0.1, 0.0025);
    alt_filter.set_parameter(0.005, 0.0025);
    x_vel_filter.set_parameter(0.01, 0.0025);
    y_vel_filter.set_parameter(0.01, 0.0025);
}

float sensor_read(void) {
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
    float filterd_v;
    static float dp, dq, dr;
    static uint16_t dcnt = 0u;
    int16_t deff;
    static int16_t old_range[4]    = {0};
    static float alt_time          = 0.0f;
    static float sensor_time       = 0.0f;
    static float old_alt_time      = 0.0f;
    static uint8_t first_flag      = 0;
    static uint8_t preMode         = 0;
    static uint8_t outlier_counter = 0;
    const uint8_t interval         = 400 / 30 + 1;
    float old_sensor_time          = 0.0;
    uint32_t st;
    float sens_interval;
    float h;
    static float opt_interval = 0.0;
    st                        = micros();
    old_sensor_time           = sensor_time;
    sensor_time               = (float)st * 1.0e-6;
    sens_interval             = sensor_time - old_sensor_time;
    opt_interval              = opt_interval + sens_interval;
    imu_update();
    acc_x          = imu_get_acc_x();
    acc_y          = imu_get_acc_y();
    acc_z          = imu_get_acc_z();
    gyro_x         = imu_get_gyro_x();
    gyro_y         = imu_get_gyro_y();
    gyro_z         = imu_get_gyro_z();
    Accel_x_raw    = acc_y;
    Accel_y_raw    = acc_x;
    Accel_z_raw    = -acc_z;
    Roll_rate_raw  = gyro_y;
    Pitch_rate_raw = gyro_x;
    Yaw_rate_raw   = -gyro_z;
    if ((Mode == PARKING_MODE) && (Mode != preMode)) {
        first_flag   = 0;
        old_range[0] = 0;
        old_range[1] = 0;
        old_range[2] = 0;
        old_range[3] = 0;
        raw_ax_filter.reset();
        raw_ay_filter.reset();
        raw_az_filter.reset();
        raw_az_d_filter.reset();
        raw_gx_filter.reset();
        raw_gy_filter.reset();
        raw_gz_filter.reset();
        az_filter.reset();
        alt_filter.reset();
        acc_filter.reset();
        x_vel_filter.reset();
        y_vel_filter.reset();
    }
    if (Mode > AVERAGE_MODE) {
        Accel_x    = raw_ax_filter.update(Accel_x_raw, Interval_time);
        Accel_y    = raw_ay_filter.update(Accel_y_raw, Interval_time);
        Accel_z    = raw_az_filter.update(Accel_z_raw, Interval_time);
        Accel_z_d  = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);
        Roll_rate  = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, Interval_time);
        Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, Interval_time);
        Yaw_rate   = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, Interval_time);
        Drone_ahrs.updateIMU((Pitch_rate) * (float)RAD_TO_DEG, (Roll_rate) * (float)RAD_TO_DEG,
                             -(Yaw_rate) * (float)RAD_TO_DEG, Accel_y, Accel_x, -Accel_z);
        Roll_angle  = Drone_ahrs.getPitch() * (float)DEG_TO_RAD;
        Pitch_angle = Drone_ahrs.getRoll() * (float)DEG_TO_RAD;
        Yaw_angle   = -Drone_ahrs.getYaw() * (float)DEG_TO_RAD;
        Az          = az_filter.update(-Accel_z_d, sens_interval);
        if (dcnt > interval) {
            if (ToF_bottom_data_ready_flag) {
                dcnt                       = 0u;
                old_alt_time               = alt_time;
                alt_time                   = micros() * 1.0e-6;
                h                          = alt_time - old_alt_time;
                ToF_bottom_data_ready_flag = 0;
                RawRange                   = tof_bottom_get_range();
                if (Mode == PARKING_MODE) RawRangeFront = tof_front_get_range();
                if (RawRange > 20) {
                    Range = RawRange;
                }
                if (RawRangeFront > 0.01) {
                    RangeFront = RawRangeFront;
                }
                deff = Range - old_range[1];
                if (deff > 500 && outlier_counter < 2) {
                    Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                } else if (deff < -500 && outlier_counter < 2) {
                    Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                } else {
                    outlier_counter = 0;
                    old_range[3]    = old_range[2];
                    old_range[2]    = old_range[1];
                    old_range[1]    = Range;
                }
                // Optical flow velocity calculation
                opt_read();
                float delta_x = getDeltaX();
                float delta_y = getDeltaY();
                // Convert pixel displacement to velocity (m/s)
                // PMW3901: 35x35 pixels, 42 deg FOV, assume 400 Hz update rate
                const float fov_rad    = 42.0f * PI / 180.0f;
                const float pixel_size = 2.0f * Altitude2 * tan(fov_rad / 2.0f) / 35.0f;  // meters per pixel
                float vx_sensor        = -delta_y * pixel_size / sens_interval;  // Sensor X (right) to body X (forward)
                float vy_sensor        = -delta_x * pixel_size / sens_interval;  // Sensor Y (forward) to body Y (right)
                // Transform to body frame using yaw angle
                float yaw  = Yaw_angle;
                X_velocity = x_vel_filter.update(vx_sensor * cos(yaw) + vy_sensor * sin(yaw), Interval_time);
                Y_velocity = y_vel_filter.update(-vx_sensor * sin(yaw) + vy_sensor * cos(yaw), Interval_time);
            }
        } else
            dcnt++;
        Altitude = alt_filter.update((float)Range / 1000.0, Interval_time);
        if (first_flag == 1)
            EstimatedAltitude.update(Altitude, Az, Interval_time);
        else
            first_flag = 1;
        Altitude2 = EstimatedAltitude.Altitude;
        if ((Altitude2 > ALT_LIMIT && Alt_flag >= 1 && Flip_flag == 0) || RawRange == 0)
            Range0flag++;
        else
            Range0flag = 0;
        if (Range0flag > RNAGE0FLAG_MAX) Range0flag = RNAGE0FLAG_MAX;
        Alt_velocity = EstimatedAltitude.Velocity;
        Az_bias      = EstimatedAltitude.Bias;
    }
    acc_norm = sqrt(Accel_x * Accel_x + Accel_y * Accel_y + Accel_z_d * Accel_z_d);
    Acc_norm = acc_filter.update(acc_norm, Control_period);
    if (Acc_norm > 2.0) {
        OverG_flag = 1;
        if (Over_g == 0.0) Over_g = acc_norm;
    }
    Voltage   = ina3221.getVoltage(INA3221_CH2);
    filterd_v = voltage_filter.update(Voltage, Control_period);
    if (Under_voltage_flag != UNDER_VOLTAGE_COUNT) {
        if (filterd_v < POWER_LIMIT)
            Under_voltage_flag++;
        else
            Under_voltage_flag = 0;
        if (Under_voltage_flag > UNDER_VOLTAGE_COUNT) Under_voltage_flag = UNDER_VOLTAGE_COUNT;
    }
    preMode     = Mode;
    uint32_t et = micros();
    return (et - st) * 1.0e-6;
}