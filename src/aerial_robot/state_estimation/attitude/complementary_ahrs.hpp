#pragma once

#include <BasicLinearAlgebra.h>

#define DELTA_T 0.01f
#define GYR_CMPF_FACTOR 100.0f
#define GYR_CMPFM_FACTOR 30.0f
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define PRESCLAER_ACC 3 // if value=1, it means same rate with gyro, for genral attitude estimation

#define G_MIN 0.7225f // 0.85^2
#define G_MAX 1.3225f // 1.15^2


class ComplementaryAHRS
{
public:
  ComplementaryAHRS()
  {
    est_g_ = {0.0, 0.0, 1.0};
    est_m_ = {0.0, 0.0, 0.0};
    prev_mag_ = {0.0, 0.0, 0.0};
    rpy_ = {0.0, 0.0, 0.0};
  }
  ~ComplementaryAHRS() = default;

  void setAcc(BLA::Matrix<3, 1> acc) {acc_ = acc;}
  void setGyro(BLA::Matrix<3, 1> gyro) {gyro_ = gyro;}
  void setMag(BLA::Matrix<3, 1> mag) {mag_ = mag;}

  BLA::Matrix<3, 1> getAcc() {return acc_;}
  BLA::Matrix<3, 1> getGyro() {return gyro_;}
  BLA::Matrix<3, 1> getMag() {return mag_;}
  BLA::Matrix<3, 1> getRpy() {return rpy_;}
  BLA::Matrix<3, 1> getEstG() {return est_g_;}
  BLA::Matrix<3, 1> getEstM() {return est_m_;}

  void estimation()
  {
    int valid_acc = 0;
    static int cnt = 0;

    float acc_magnitude = acc_(0) * acc_(0) + acc_(1) * acc_(1) + acc_(2) * acc_(2);
    BLA::Matrix<3, 1> est_g_tmp = est_g_;
    BLA::Matrix<3, 1> est_m_tmp = est_m_;

    BLA::Matrix<3, 1> gyro_rotate = gyro_ * DELTA_T;

    est_m_ = est_m_ + CrossProduct(est_m_tmp, gyro_rotate); //rotation by gyro
    est_g_ = est_g_ + CrossProduct(est_g_tmp, gyro_rotate); //rotation by gyro

    if(G_MIN < acc_magnitude && acc_magnitude < G_MAX) valid_acc = 1;

    est_g_tmp = est_g_;
    est_m_tmp = est_m_;

    /* acc correction */
    if(valid_acc == 1 && cnt == 0)
      est_g_ = (est_g_tmp * GYR_CMPF_FACTOR + acc_) * INV_GYR_CMPF_FACTOR;

    /* mag correction */
    if(prev_mag_(0) != mag_(0) || prev_mag_(1) != mag_(1) || prev_mag_(2) != mag_(2))
      {
        prev_mag_ = mag_;
        est_m_ = (est_m_tmp * GYR_CMPFM_FACTOR  + mag_) * INV_GYR_CMPFM_FACTOR;
      }

    // Attitude of the estimated vector
    float sq_g_x_sq_g_z = est_g_(0) * est_g_(0) + est_g_(2) * est_g_(2);
    float sq_g_y_sq_g_z = est_g_(1) * est_g_(1) + est_g_(2) * est_g_(2);
    float invG = invSqrt(sq_g_x_sq_g_z + est_g_(1) * est_g_(1));

    rpy_(0) = atan2f(est_g_(1) , est_g_(2));
    rpy_(1) = atan2f(-est_g_(0) , invSqrt(sq_g_y_sq_g_z) * sq_g_y_sq_g_z);
    rpy_(2) = atan2f(est_m_(2) * est_g_(1) - est_m_(1) * est_g_(2),
                     est_m_(0) * invG * sq_g_y_sq_g_z  - (est_m_(1) * est_g_(1) + est_m_(2) * est_g_(2)) * invG * est_g_(0)); // + mag_declination

    /* update */
    if(valid_acc) cnt++;
    if(cnt == PRESCLAER_ACC) cnt = 0;
  }

private:
  BLA::Matrix<3, 1> gyro_;
  BLA::Matrix<3, 1> acc_;
  BLA::Matrix<3, 1> mag_;
  BLA::Matrix<3, 1> prev_mag_;
  BLA::Matrix<3, 1> rpy_;
  BLA::Matrix<3, 1> est_g_;
  BLA::Matrix<3, 1> est_m_;

  float invSqrt(float x)
  {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
  }
};
