#ifndef POSITION_KALMAN_HPP
#define POSITION_KALMAN_HPP

#include "tiny_ekf.h"

class PositionEKF : public TinyEKF {
public:
    PositionEKF();
    void set_dt(float dt);
    void set_ax_ay(float ax, float ay);
    void update_with_time(float current_time, double z[2]);
    float get_x() { return static_cast<float>(x[0]); }
    float get_y() { return static_cast<float>(x[1]); }
    float get_vx() { return static_cast<float>(x[2]); }
    float get_vy() { return static_cast<float>(x[3]); }
    void reset();

protected:
    float dt;
    float ax_world, ay_world;
    float last_time;

    void model(double fx[4], double F[4][4], double hx[2], double H[2][4]);
};

#endif