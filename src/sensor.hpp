#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <cstdint>
#include <Arduino.h>
#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include <MadgwickAHRS.h>
#include <common.h>
#include <stdint.h>
#include "alt_kalman.hpp"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

#define SDA_PIN      (3)
#define SCL_PIN      (4)
#define PIN_NUM_MISO (43)
#define PIN_NUM_MOSI (14)
#define PIN_NUM_CLK  (44)
#define PIN_CS       (46)

typedef struct {
    spi_host_device_t host;  ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;        ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;      ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;  ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling
                     ///< `spi_eeprom_init()`.
} eeprom_config_t;

typedef struct eeprom_context_t* eeprom_handle_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quat_t;

typedef struct {
    uint16_t distance;
    uint16_t cnt;
} distance_t;
extern volatile float Roll_angle, Pitch_angle, Yaw_angle;
extern volatile float Roll_rate, Pitch_rate, Yaw_rate;
extern volatile float Roll_rate_offset, Pitch_rate_offset, Yaw_rate_offset;
extern volatile float Accel_z_d;
extern volatile float Accel_z_offset;
extern volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
extern volatile float Accel_x, Accel_y, Accel_z;
extern volatile float Accel_z_d;
extern volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
extern volatile float Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
extern volatile int16_t RawRange;
extern volatile int16_t Range;
extern volatile int16_t RawRangeFront;
extern volatile int16_t RangeFront;
extern volatile float Altitude;
extern volatile float Altitude2;
extern volatile float Alt_velocity;
extern volatile uint8_t Alt_control_ok;
extern quat_t Quat;
extern volatile uint8_t Under_voltage_flag;
extern volatile uint8_t ToF_bottom_data_ready_flag;
extern volatile float Az;
extern volatile float Az_bias;
extern Alt_kalman EstimatedAltitude;
extern volatile int16_t RawRangeFront;
extern volatile int16_t RangeFront;
extern volatile int16_t deltaX, deltaY;
extern volatile float Voltage;
extern volatile float Acc_norm;
extern volatile float Over_g, Over_rate;
extern volatile uint8_t OverG_flag;
extern volatile uint8_t Range0flag;
extern volatile uint8_t Under_voltage_flag;
extern volatile uint16_t Offset_counter;

// Optical flow data for EKF
extern volatile float dx; // Optical flow velocity (x-axis, pixels)
extern volatile float dy; // Optical flow velocity (y-axis, pixels)

void sensor_init(void);
float sensor_read(void);
void sensor_reset_offset(void);
void sensor_calc_offset_avarage(void);
void ahrs_reset(void);
uint8_t scan_i2c(void);

#endif // SENSOR_HPP