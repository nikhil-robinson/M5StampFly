#include "pos_kalman.hpp"
#include <cmath>
#include <cstring>

PositionEKF::PositionEKF() {
    std::memset(state, 0, sizeof(state));
    std::memset(P, 0, sizeof(P));
    for (int i = 0; i < 4; ++i) {
        P[i][i] = 1000.0f;
    }
    std::memset(Q, 0, sizeof(Q));
    Q[2][2] = 0.1f;  // Variance for vx
    Q[3][3] = 0.1f;  // Variance for vy
    std::memset(R, 0, sizeof(R));
    R[0][0] = 1.0f;  // Variance for dx
    R[1][1] = 1.0f;  // Variance for dy
}

void PositionEKF::predict(float ax_body, float ay_body, float az_body, float roll, float pitch, float yaw, float dt) {
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                     {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                     {-sp, cp * sr, cp * cr}};

    float a_true[3] = {ax_body - R[2][0] * GRAVITY, ay_body - R[2][1] * GRAVITY, az_body - R[2][2] * GRAVITY};

    float ax_world = R[0][0] * a_true[0] + R[0][1] * a_true[1] + R[0][2] * a_true[2];
    float ay_world = R[1][0] * a_true[0] + R[1][1] * a_true[1] + R[1][2] * a_true[2];

    float x  = state[0];
    float y  = state[1];
    float vx = state[2];
    float vy = state[3];

    state[0] = x + vx * dt + 0.5f * ax_world * dt * dt;
    state[1] = y + vy * dt + 0.5f * ay_world * dt * dt;
    state[2] = vx + ax_world * dt;
    state[3] = vy + ay_world * dt;

    float F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    float temp[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            temp[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                temp[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                P[i][j] += temp[i][k] * F[j][k];
            }
            P[i][j] += Q[i][j];
        }
    }
}

void PositionEKF::update(float dx, float dy, float height, float dt) {
    float h[2];
    h[0] = -(FOCAL_LENGTH * state[2] * dt) / height;
    h[1] = -(FOCAL_LENGTH * state[3] * dt) / height;

    float y[2] = {dx - h[0], dy - h[1]};

    float H[2][4] = {{0, 0, -(FOCAL_LENGTH * dt) / height, 0}, {0, 0, 0, -(FOCAL_LENGTH * dt) / height}};

    float temp[2][4];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 4; ++j) {
            temp[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                temp[i][j] += H[i][k] * P[k][j];
            }
        }
    }
    float S[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            S[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                S[i][j] += temp[i][k] * H[j][k];
            }
            S[i][j] += R[i][j];
        }
    }

    float Ht[4][2];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            Ht[i][j] = H[j][i];
        }
    }

    float temp2[4][2];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            temp2[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                temp2[i][j] += P[i][k] * Ht[k][j];
            }
        }
    }

    float det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabs(det_S) < 1e-6) return;
    float S_inv[2][2] = {{S[1][1] / det_S, -S[0][1] / det_S}, {-S[1][0] / det_S, S[0][0] / det_S}};

    float K[4][2];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            K[i][j] = 0;
            for (int k = 0; k < 2; ++k) {
                K[i][j] += temp2[i][k] * S_inv[k][j];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        state[i] += K[i][0] * y[0] + K[i][1] * y[1];
    }

    float KH[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            KH[i][j] = 0;
            for (int k = 0; k < 2; ++k) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    float I_KH[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    float new_P[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            new_P[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                new_P[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    std::memcpy(P, new_P, sizeof(P));
}

void PositionEKF::get_state(float& x, float& y, float& vx, float& vy) {
    x  = state[0];
    y  = state[1];
    vx = state[2];
    vy = state[3];
}