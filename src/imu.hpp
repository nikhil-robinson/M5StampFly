#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "bmi2.h"
#include "bmi2.h"
#include <bmi270.h>
#include "spi_s3.hpp"

#define DPS20002RAD   34.90658504
#define DPS10002RAD   17.4532925199
#define ACCEL         UINT8_C(0x00)
#define GYRO          UINT8_C(0x01)
#define GRAVITY_EARTH (9.80665f)

void imu_init(void);
void imu_test(void);
void imu_update(void);
float imu_get_acc_x(void);
float imu_get_acc_y(void);
float imu_get_acc_z(void);
float imu_get_gyro_x(void);
float imu_get_gyro_y(void);
float imu_get_gyro_z(void);

float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
float lsb_to_rps(int16_t val, float rps, uint8_t bit_width);

void bmi270_dev_init(void);
void bmi2_delay_us(uint32_t period, void *intf_ptr);
int8_t set_accel_gyro_config(struct bmi2_dev *bmi);
void bmi2_error_codes_print_result(int8_t rslt);

#endif
