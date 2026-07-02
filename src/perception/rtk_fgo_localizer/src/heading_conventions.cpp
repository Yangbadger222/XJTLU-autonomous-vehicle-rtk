#include "rtk_fgo_localizer/heading_conventions.hpp"

#include "rtk_fgo_localizer/correction_smoother.hpp"

#include <cmath>
#include <limits>

namespace rtk_fgo_localizer
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

double compassHeadingDegToEnuYawRad(double heading_deg)
{
  if (!std::isfinite(heading_deg)) {
    return nan();
  }
  return normalizeYaw((90.0 - heading_deg) * kPi / 180.0);
}

double headingQuaternionYawToEnuYaw(double quaternion_yaw_rad, bool quaternion_yaw_is_compass)
{
  if (!std::isfinite(quaternion_yaw_rad)) {
    return nan();
  }
  if (!quaternion_yaw_is_compass) {
    return normalizeYaw(quaternion_yaw_rad);
  }
  return compassHeadingDegToEnuYawRad(quaternion_yaw_rad * 180.0 / kPi);
}

}  // namespace rtk_fgo_localizer
