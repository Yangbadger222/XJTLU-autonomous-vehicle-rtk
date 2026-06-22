#include "rtk_fgo_localizer/correction_smoother.hpp"

#include <algorithm>
#include <cmath>

namespace rtk_fgo_localizer
{

CorrectionSmoother::CorrectionSmoother(
  double max_translation_step_m,
  double max_yaw_step_rad)
: max_translation_step_m_(std::max(0.0, max_translation_step_m)),
  max_yaw_step_rad_(std::max(0.0, max_yaw_step_rad))
{
}

Correction2D CorrectionSmoother::step(const Correction2D & requested) const
{
  Correction2D out = requested;

  const double translation_norm = std::hypot(requested.dx, requested.dy);
  if (translation_norm > max_translation_step_m_ && translation_norm > 0.0) {
    const double scale = max_translation_step_m_ / translation_norm;
    out.dx = requested.dx * scale;
    out.dy = requested.dy * scale;
  }

  const double normalized_yaw = normalizeYaw(requested.dyaw);
  out.dyaw = std::clamp(normalized_yaw, -max_yaw_step_rad_, max_yaw_step_rad_);
  return out;
}

double normalizeYaw(double yaw_rad)
{
  return std::atan2(std::sin(yaw_rad), std::cos(yaw_rad));
}

}  // namespace rtk_fgo_localizer
