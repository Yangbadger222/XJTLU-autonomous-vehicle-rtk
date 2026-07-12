#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace serial_twistctl {

inline std::string formatTwistCommand(
    double linear_x,
    double angular_z,
    double angular_z_scale) {
  char command[50];
  std::snprintf(
      command,
      sizeof(command),
      "vcx=%.3f,wc=%.3f\n",
      static_cast<float>(linear_x),
      static_cast<float>(angular_z * angular_z_scale));
  return std::string(command);
}

inline double limitCommandAcceleration(
    double previous,
    double target,
    double max_increase) {
  if (!std::isfinite(previous) || !std::isfinite(target) ||
      !std::isfinite(max_increase) || max_increase <= 0.0) {
    return 0.0;
  }
  if (target == 0.0) {
    return 0.0;
  }
  if (previous * target < 0.0) {
    return 0.0;
  }
  if (previous == 0.0) {
    return std::copysign(std::min(std::abs(target), max_increase), target);
  }
  if (std::abs(target) <= std::abs(previous)) {
    return target;
  }
  const double increase = std::min(
      std::abs(target) - std::abs(previous), max_increase);
  return previous + std::copysign(increase, target);
}

}  // namespace serial_twistctl
