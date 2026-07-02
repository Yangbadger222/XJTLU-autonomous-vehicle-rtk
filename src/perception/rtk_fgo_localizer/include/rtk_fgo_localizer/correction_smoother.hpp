#pragma once

namespace rtk_fgo_localizer
{

struct Correction2D
{
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
};

class CorrectionSmoother
{
public:
  CorrectionSmoother(double max_translation_step_m, double max_yaw_step_rad);

  Correction2D step(const Correction2D & requested) const;

private:
  double max_translation_step_m_ = 0.15;
  double max_yaw_step_rad_ = 0.005235987755982989;
};

double normalizeYaw(double yaw_rad);

}  // namespace rtk_fgo_localizer
