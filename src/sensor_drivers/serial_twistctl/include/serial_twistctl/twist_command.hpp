#pragma once

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

}  // namespace serial_twistctl
