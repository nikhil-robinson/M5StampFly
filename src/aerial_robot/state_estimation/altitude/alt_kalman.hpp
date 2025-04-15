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

#ifndef ALT_KALMAN_HPP
#define ALT_KALMAN_HPP

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

class Alt_kalman {
  // state
  BLA::Matrix<3, 1> estimate_state_;
  BLA::Matrix<3, 1> predict_state_;
  BLA::Matrix<3, 1> control_input_model_;
  BLA::Matrix<3, 3> state_transition_model_;
  BLA::Matrix<3, 3> state_transition_model_transpose_;
  BLA::Matrix<3, 3> predict_P_;
  BLA::Matrix<3, 3> correction_P_;
  BLA::Matrix<3, 3> Q_;
  BLA::Matrix<3, 3> R_;
  BLA::Matrix<3, 3> H_;
  BLA::Matrix<3, 3> G_;

  float gravity_ = 9.80665;

  // Sensor
  // float z_sens;

  // Bias beta
  float beta = -0.01;

  // Q
  float q1 = 0.1 * 0.1, q2 = (1.0) * (1.0);  // q1=1.0*1.0 q2=1.0*1.0

  // R
  // float R = 0.004*0.004;
  float R = 0.004 * 0.004;

public:
  // step
  float step = 1.0 / 400.0;
  // state
  float Velocity = 0.0, Altitude = 0.0, Bias = 0.0;

  // Method
  Alt_kalman();
  void initialize();
  void update(float z_sens, float accel, float h);
  void reset(void);
};

#endif
