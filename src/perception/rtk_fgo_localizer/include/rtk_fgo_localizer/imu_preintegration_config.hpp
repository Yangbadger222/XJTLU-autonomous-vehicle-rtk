#pragma once

namespace rtk_fgo_localizer
{

struct ImuPreintegrationConfig
{
  double accelerometer_noise_sigma = 0.1;
  double gyroscope_noise_sigma = 0.01;
  double accelerometer_bias_rw_sigma = 0.001;
  double gyroscope_bias_rw_sigma = 0.0001;
  double integration_error_sigma = 1e-8;
  double gravity_mps2 = 9.81;
};

}  // namespace rtk_fgo_localizer
