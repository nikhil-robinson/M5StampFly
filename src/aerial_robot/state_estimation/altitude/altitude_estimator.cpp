#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>

void AltitudeEstimator::initialize()
{
  altitude_kalman_filter_.initialize();

  raw_az_filter_.set_parameter(0.003, 0.0025);
  raw_az_d_filter_.set_parameter(0.1, 0.0025);  // alt158
  az_filter_.set_parameter(0.1, 0.0025);        // alt158

  tof_init();

  uint16_t cnt = 0;
  while (cnt < 10) {
    if (ToF_bottom_data_ready_flag_) {
      ToF_bottom_data_ready_flag_ = 0;
      cnt++;
      USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
    }
  }
}

void AltitudeEstimator::update()
{

}
