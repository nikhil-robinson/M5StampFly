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

#include "aerial_robot/state_estimation/altitude/alt_kalman.hpp"
// #include <stdio.h>
// #include <math.h>
// #include <stdlib.h>
#include <Arduino.h>

Alt_kalman::Alt_kalman() {};

void Alt_kalman::initialize()
{
  estimate_state_ = {0.0, 0.0, 0.0};
  predict_state_ = {0.0, 0.0, 0.0};
  state_transition_model_ = {1.0, 0.0, -gravity_ * step,
                             step, 1.0, 0.0,
                             0.0, 0.0, 1 + beta * step};
  state_transition_model_transpose_ = {1.0, step, 0.0,
                                       0.0, 1.0, 0.0,
                                       -gravity_ * step, 0.0, 1.0 + beta * step};
  predict_P_ = {100.0, 0.0, 0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0, 100.0};

  correction_P_ = {100.0, 0.0, 0.0,
                   0.0, 100.0, 0.0,
                   0.0, 0.0, 100.0};

  Q_ = {q1, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, q2};

  R_ = {0.0, 0.0, 0.0,
        0.0, R, 0.0,
        0.0, 0.0, 0.0};

  H_ = {0.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 0.0};

  G_ = {step, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, step};
}

void Alt_kalman::update(float z_sens, float accel, float h)
{
  step = h;

  // predict state
  control_input_model_ = {gravity_ * accel * step, 0.0, 0.0};
  predict_state_ = state_transition_model_ * estimate_state_ + control_input_model_;

  // predict P
  predict_P_ = state_transition_model_ * correction_P_ * state_transition_model_transpose_ + G_ * Q_ * G_; // G_transpose is same as G

  // update kalman gain
  // BLA::Matrix<3, 3> S = R + H_ * predict_P_ * H_; // H_transpose is same as H
  // BLA::Matrix<3, 3> K = predict_P_ * H_ * Inverse(S); // H_transpose is same as H
  float s = predict_P_(1, 1) + R_(1, 1);
  BLA::Matrix<3, 3> K = {0.0, predict_P_(0, 1) / s, 0.0,
                         0.0, predict_P_(1, 1) / s, 0.0,
                         0.0, predict_P_(2, 1) / s, 0.0};

  // inovation
  BLA::Matrix<3, 1> observation = {0.0, z_sens, 0.0};
  BLA::Matrix<3, 1> e = observation - H_ * estimate_state_;

  // estimate state
  estimate_state_ = predict_state_ + K * e;

  // Estimated state output
  Velocity = estimate_state_(0);
  Altitude = estimate_state_(1);
  Bias = estimate_state_(2);

  // estimate P
  correction_P_ = (BLA::Eye<3, 3>() - K * H_) * predict_P_;
}

void Alt_kalman::reset(void)
{
  // P
  predict_P_ = {100.0, 0.0, 0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0, 100.0};

  correction_P_ = {100.0, 0.0, 0.0,
                   0.0, 100.0, 0.0,
                   0.0, 0.0, 100.0};

  Velocity = 0.0, Altitude = 0.0, Bias = 0.0;
}
