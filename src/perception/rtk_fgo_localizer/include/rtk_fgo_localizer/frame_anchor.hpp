#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <memory>
#include <optional>
#include <string>

namespace rtk_fgo_localizer
{

struct GeoPoint
{
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

class FrameAnchor
{
public:
  ~FrameAnchor();

  bool initialize(
    const GeoPoint & origin_fix,
    const gtsam::Pose3 & reference_fgo_pose,
    std::optional<double> rtk_heading_yaw_rad);

  bool initialized() const;
  std::optional<gtsam::Point3> geoToMap(const GeoPoint & fix) const;
  gtsam::Point3 enuToMap(const gtsam::Point3 & enu) const;
  std::string statusString() const;

private:
  bool initialized_ = false;
  bool yaw_from_heading_ = false;
  GeoPoint origin_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  gtsam::Rot3 map_R_enu_;
  gtsam::Point3 map_t_enu_;
};

}  // namespace rtk_fgo_localizer
