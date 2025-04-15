#pragma once

#include <algorithm>

class PIDController
{
public:
  PIDController(const float p_gain, const float i_gain, const float d_gain,
                const float limit_sum = 1e6, const float limit_p = 1e6, const float limit_i = 1e6, const float limit_d = 1e6,
                const float limit_err_p = 1e6, const float limit_err_i = 1e6, const float limit_err_d = 1e6):
    result_(0), err_p_(0), err_i_(0), err_i_prev_(0), err_d_(0),
    p_term_(0), i_term_(0), d_term_(0)
  {
    setGains(p_gain, i_gain, d_gain);
    setLimits(limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d);
  }

  ~PIDController() = default;

  void update(const float err_p, const float du, const float err_d)
  {
    err_p_ = clamp(err_p, -limit_err_p_, limit_err_p_);
    err_i_prev_ = err_i_;
    err_i_ = clamp(err_i_ + err_p_ * du, -limit_err_i_, limit_err_i_);
    err_d_ = clamp(err_d, -limit_err_d_, limit_err_d_);

    p_term_ = clamp(err_p_ * p_gain_, -limit_p_, limit_p_);
    i_term_ = clamp(err_i_ * i_gain_, -limit_i_, limit_i_);
    d_term_ = clamp(err_d_ * d_gain_, -limit_d_, limit_d_);

    result_ = clamp(p_term_ + i_term_ + d_term_, -limit_sum_, limit_sum_);
  }

  void reset()
  {
    err_i_ = 0;
    err_i_prev_ = 0;
    result_ = 0;
  }

  void setPGain(const float p_gain) { p_gain_ = p_gain; }
  void setIGain(const float i_gain) { i_gain_ = i_gain; }
  void setDGain(const float d_gain) { d_gain_ = d_gain; }
  void setGains(const float p_gain, const float i_gain, const float d_gain)
  {
    setPGain(p_gain);
    setIGain(i_gain);
    setDGain(d_gain);
  }

  void setLimitSum(const float limit_sum) {limit_sum_ = limit_sum; }
  void setLimitP(const float limit_p) {limit_p_ = limit_p; }
  void setLimitI(const float limit_i) {limit_i_ = limit_i; }
  void setLimitD(const float limit_d) {limit_d_ = limit_d; }
  void setLimitErrP(const float limit_err_p) {limit_err_p_ = limit_err_p; }
  void setLimitErrI(const float limit_err_i) {limit_err_i_ = limit_err_i; }
  void setLimitErrD(const float limit_err_d) {limit_err_d_ = limit_err_d; }

  void setLimits(const float limit_sum, const float limit_p, const float limit_i, const float limit_d, const float limit_err_p, const float limit_err_i, const float limit_err_d)
  {
    setLimitSum(limit_sum);
    setLimitP(limit_p);
    setLimitI(limit_i);
    setLimitD(limit_d);
    setLimitErrP(limit_err_p);
    setLimitErrI(limit_err_i);
    setLimitErrD(limit_err_d);
  }

private:
  float clamp(float value, float min_val, float max_val) {return std::max(min_val, std::min(value, max_val));}

  float p_gain_, i_gain_, d_gain_;
  float p_term_, i_term_, d_term_;
  float result_;

  float err_p_, err_i_, err_i_prev_, err_d_;
  float limit_sum_, limit_p_, limit_i_, limit_d_;
  float limit_err_p_, limit_err_i_, limit_err_d_;
};
