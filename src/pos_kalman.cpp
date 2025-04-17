#include "position_kalman.hpp"

PositionEKF::PositionEKF() : TinyEKF(4, 2) {
    last_time = 0.0f;
    // Set process noise (Q)
    setQ(0, 0, 0.01); // x
    setQ(1, 1, 0.01); // y
    setQ(2, 2, 0.1);  // vx
    setQ(3, 3, 0.1);  // vy
    // Set measurement noise (R)
    setR(0, 0, 0.5);  // vx
    setR(1, 1, 0.5);  // vy
}

void PositionEKF::set_dt(float dt) {
    this->dt = dt;
}

void PositionEKF::set_ax_ay(float ax, float ay) {
    ax_world = ax;
    ay_world = ay;
}

void PositionEKF::update_with_time(float current_time, double z[2]) {
    if (last_time == 0.0f) {
        last_time = current_time;
        return;
    }
    dt = current_time - last_time;
    step(z);
    last_time = current_time;
}

void PositionEKF::reset() {
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;
    x[3] = 0.0;
    // P is automatically reset in TinyEKF
}

void PositionEKF::model(double fx[4], double F[4][4], double hx[2], double H[2][4]) {
    // State transition
    fx[0] = x[0] + x[2]*dt + 0.5*ax_world*dt*dt;
    fx[1] = x[1] + x[3]*dt + 0.5*ay_world*dt*dt;
    fx[2] = x[2] + ax_world*dt;
    fx[3] = x[3] + ay_world*dt;

    // F matrix
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F[i][j] = 0.0;
        }
    }
    F[0][0] = 1;
    F[0][2] = dt;
    F[1][1] = 1;
    F[1][3] = dt;
    F[2][2] = 1;
    F[3][3] = 1;

    // Measurement function
    hx[0] = x[2]; // vx
    hx[1] = x[3]; // vy

    // H matrix
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            H[i][j] = 0.0;
        }
    }
    H[0][2] = 1;
    H[1][3] = 1;
}