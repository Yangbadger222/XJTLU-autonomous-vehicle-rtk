#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>

#include "fgo_gil_localizer/math_types.hpp"

namespace fgo_gil_localizer
{

struct ReceiverRtkQuality
{
  int quality = 0;
  int satellites = 0;
  double hdop = 0.0;
};

std::optional<ReceiverRtkQuality> parseGgaRtkQuality(const std::string & sentence);

bool isAcceptedFixedRtk(
  const ReceiverRtkQuality & quality,
  int fixed_quality_code,
  int minimum_satellites,
  double maximum_hdop);

std::optional<Vec3> geodeticToEcef(
  double latitude_deg,
  double longitude_deg,
  double ellipsoid_altitude_m);

std::optional<Eigen::Matrix3d> enuCovarianceToEcef(
  double latitude_deg,
  double longitude_deg,
  const Eigen::Matrix3d & covariance_enu_m2);

}  // namespace fgo_gil_localizer
