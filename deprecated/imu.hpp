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

#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "bmi2.h"
#include "bmi2.h"
#include <bmi270.h>
#include "spi_s3.hpp"

#define DPS20002RAD 34.90658504
#define DPS10002RAD 17.4532925199
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define GRAVITY_EARTH  (9.80665f)

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
