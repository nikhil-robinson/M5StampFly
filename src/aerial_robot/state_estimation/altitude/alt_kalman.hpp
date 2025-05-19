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
   public:
    // Method
    Alt_kalman();
    void initialize();
    void update(float z_sens, float accel);
    void reset(void);
    float getAltitude() {
        return altitude_;
    }
    float getVelocity() {
        return velocity_;
    }

    float velocity_ = 0.0;
    float altitude_ = 0.0;
    float bias_     = 0.0;

   private:
    BLA::Matrix<3, 1> estimate_state_;
    BLA::Matrix<3, 1> predict_state_;
    BLA::Matrix<3, 1> control_input_model_;
    BLA::Matrix<3, 3> state_transition_model_;
    BLA::Matrix<3, 3> state_transition_model_transpose_;
    BLA::Matrix<3, 3> predict_P_;
    BLA::Matrix<3, 3> correction_P_;
    BLA::Matrix<2, 2> Q_;
    BLA::Matrix<3, 2> G_;
    BLA::Matrix<2, 3> G_transpose_;
    BLA::Matrix<1, 1> R_;
    BLA::Matrix<1, 3> H_;
    BLA::Matrix<3, 1> H_transpose_;

    float gravity_ = 9.80665;

    float beta = -0.01;

    float step_ = 1.0 / 500.0;

    int loop_count_ = 0;
};
#endif
