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
#include <Arduino.h>

Alt_kalman::Alt_kalman(){};

void Alt_kalman::initialize() {
    estimate_state_ = BLA::Zeros<3, 1>();
    predict_state_  = BLA::Zeros<3, 1>();

    state_transition_model_ = {1.0, 0.0, -step_, step_, 1.0, 0.0, 0.0, 0.0, 1 + beta * step_};

    state_transition_model_transpose_ = {1.0, step_, 0.0, 0.0, 1.0, 0.0, -step_, 0.0, 1 + beta * step_};

    predict_P_    = (float)100.0 * BLA::Eye<3, 3>();
    correction_P_ = (float)100.0 * BLA::Eye<3, 3>();

    Q_ = {0.1, 0.0, 0.0, 0.1};

    G_ = {step_, 0.0, 0.0, 0.0, 0.0, step_};

    G_transpose_ = {step_, 0.0, 0.0, 0.0, 0.0, step_};

    R_ = {0.0001};

    H_           = {0.0, 1.0, 0.0};
    H_transpose_ = {0.0, 1.0, 0.0};
}

void Alt_kalman::update(float z_sens, float accel) {
    loop_count_++;

    // predict state
    control_input_model_ = {accel * step_, 0.0, 0.0};
    predict_state_       = state_transition_model_ * estimate_state_ + control_input_model_;

    // predict P
    predict_P_ = state_transition_model_ * correction_P_ * state_transition_model_transpose_ + G_ * Q_ * G_transpose_;

    BLA::Matrix<1, 1> observation = {z_sens};
    BLA::Matrix<1, 1> e           = observation - H_ * predict_state_;

    BLA::Matrix<1, 1> S = R_ + H_ * predict_P_ * H_transpose_;
    BLA::Matrix<3, 1> K = predict_P_ * H_transpose_ * Inverse(S);

    // estimate state
    estimate_state_ = predict_state_ + K * e;

    // Estimated state output
    velocity_ = estimate_state_(0);
    altitude_ = estimate_state_(1);
    bias_     = estimate_state_(2);

    // estimate P
    correction_P_ = (BLA::Eye<3, 3>() - K * H_) * predict_P_;
}

void Alt_kalman::reset(void) {
    // P
    predict_P_ = {100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0};

    correction_P_ = {100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0};

    velocity_ = 0.0;
    altitude_ = 0.0;
    bias_     = 0.0;
}
