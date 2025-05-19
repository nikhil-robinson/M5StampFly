#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>

class ExtendedKalmanFilter {
   public:
    ExtendedKalmanFilter(){};
    ~ExtendedKalmanFilter() = default;

    void initialize();
    void update(float x_vel, float y_vel, float tof, float acc_x, float acc_y, float acc_z, float omega_x,
                float omega_y, float omega_z);

    void reset();

    void setRotationMatrix(BLA::Matrix<3, 3> rotation_matrix) {
        rotation_matrix_ = rotation_matrix;
    }

    BLA::Matrix<3, 1> getPos() {
        return estimate_state_.Submatrix<3, 1>(3, 0);
    }
    BLA::Matrix<3, 1> getVel() {
        return estimate_state_.Submatrix<3, 1>(0, 0);
    }

   private:
    BLA::Matrix<9, 1> estimate_state_;
    BLA::Matrix<9, 1> predict_state_;
    BLA::Matrix<9, 1> control_input_model_;
    BLA::Matrix<3, 1> observation_;
    BLA::Matrix<9, 9> state_transition_model_;
    BLA::Matrix<9, 9> state_transition_model_transpose_;
    BLA::Matrix<9, 9> predict_P_;
    BLA::Matrix<9, 9> correction_P_;
    BLA::Matrix<3, 3> R_;
    BLA::Matrix<3, 9> H_;
    BLA::Matrix<9, 3> H_transpose_;
    BLA::Matrix<9, 6> G_;
    BLA::Matrix<6, 9> G_transpose_;
    BLA::Matrix<6, 6> Q_;
    BLA::Matrix<3, 3> rotation_matrix_;

    float gravity_ = 9.80665;
    float step_    = 1.0 / 500.0;
    float beta_    = -0.0001;

    float flow_max_ = 30;

    int loop_count_ = 0;

    void predict();
    void correct();
};
