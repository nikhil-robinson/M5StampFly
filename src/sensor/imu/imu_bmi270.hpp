#pragma once

#include <sensor/imu/imu.hpp>
#include <Arduino.h>
#include "bmi2.h"
#include "bmi2.h"
#include <bmi270.h>
#include <spi_s3.hpp>

#define DPS20002RAD 34.90658504
#define DPS10002RAD 17.4532925199
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define GRAVITY_EARTH  (9.80665f)

class ImuBMI270 : public Imu
{
public:
  ImuBMI270(){};
  ~ImuBMI270() = default;

  void initialize() override;

private:
  struct bmi2_sens_data imu_data_;
  struct bmi2_dev Bmi270_;
  struct bmi2_dev *pBmi270_ = &Bmi270_;

  void readImuData() override;

  void imu_update(void) {bmi2_get_sensor_data(&imu_data_, pBmi270_);}
  float imu_get_acc_x(void) {return lsb_to_mps2(imu_data_.acc.x, 8.0, 16) / GRAVITY_EARTH;}
  float imu_get_acc_y(void) {return lsb_to_mps2(imu_data_.acc.y, 8.0, 16) / GRAVITY_EARTH;}
  float imu_get_acc_z(void) {return lsb_to_mps2(imu_data_.acc.z, 8.0, 16) / GRAVITY_EARTH;}
  float imu_get_gyro_x(void) {return lsb_to_rps(imu_data_.gyr.x, DPS20002RAD, 16);}
  float imu_get_gyro_y(void) {return lsb_to_rps(imu_data_.gyr.y, DPS20002RAD, 16);}
  float imu_get_gyro_z(void) {return lsb_to_rps(imu_data_.gyr.z, DPS20002RAD, 16);}

  float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
  {
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
  }

  float lsb_to_rps(int16_t val, float rps, uint8_t bit_width)
  {
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (rps / (half_scale)) * (val);
  }

  void bmi270_dev_init(void);
  static void bmi2_delay_us(uint32_t period, void *intf_ptr);
  int8_t set_accel_gyro_config(struct bmi2_dev *bmi);
  void bmi2_error_codes_print_result(int8_t rslt);
};
