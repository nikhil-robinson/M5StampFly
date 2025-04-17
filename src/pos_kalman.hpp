#ifndef POSITION_EKF_H
#define POSITION_EKF_H

#include <cstdint>

class PositionEKF {
public:
    PositionEKF();
    void predict(float ax_body, float ay_body, float az_body, float roll, float pitch, float yaw, float dt);
    void update(float dx, float dy, float height, float dt);
    void get_state(float& x, float& y, float& vx, float& vy);

private:
    // State vector: [x, y, vx, vy]
    float state[4];
    // Covariance matrix
    float P[4][4];
    // Process noise covariance
    float Q[4][4];
    // Measurement noise covariance
    float R[2][2];
    // Focal length in pixels (approximated for PMW3901)
    static constexpr float FOCAL_LENGTH = 39.0f;
    // Gravity constant
    static constexpr float GRAVITY = 9.81f;
};

#endif // POSITION_EKF_H