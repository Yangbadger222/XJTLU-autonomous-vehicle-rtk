#include "rtk_fgo_localizer/frame_anchor.hpp"

#include "rtk_fgo_localizer/correction_smoother.hpp"

#include <GeographicLib/LocalCartesian.hpp>

#include <cmath>
#include <sstream>

namespace rtk_fgo_localizer
{
namespace
{

bool finiteGeoPoint(const GeoPoint & point)
{
  return std::isfinite(point.latitude_deg) &&
         std::isfinite(point.longitude_deg) &&
         std::isfinite(point.altitude_m);
}

}  // namespace

FrameAnchor::~FrameAnchor() = default;

bool FrameAnchor::initialize(
  const GeoPoint & origin_fix,
  const gtsam::Pose3 & reference_fgo_pose,
  std::optional<double> rtk_heading_yaw_rad)
{
  if (!finiteGeoPoint(origin_fix)) {
    return false;
  }
  if (rtk_heading_yaw_rad.has_value() && !std::isfinite(*rtk_heading_yaw_rad)) {
    return false;
  }

  origin_ = origin_fix;
  local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
    origin_fix.latitude_deg,
    origin_fix.longitude_deg,
    origin_fix.altitude_m);

  const double reference_yaw = reference_fgo_pose.rotation().yaw();
  if (rtk_heading_yaw_rad.has_value()) {
    map_R_enu_ = gtsam::Rot3::Yaw(reference_yaw - *rtk_heading_yaw_rad);
    yaw_from_heading_ = true;
  } else {
    map_R_enu_ = gtsam::Rot3();
    yaw_from_heading_ = false;
  }
  map_t_enu_ = reference_fgo_pose.translation();
  initialized_ = true;
  return true;
}

bool FrameAnchor::initialized() const
{
  return initialized_;
}

std::optional<gtsam::Point3> FrameAnchor::geoToMap(const GeoPoint & fix) const
{
  if (!initialized_ || !local_cartesian_ || !finiteGeoPoint(fix)) {
    return std::nullopt;
  }

  double east_m = 0.0;
  double north_m = 0.0;
  double up_m = 0.0;
  local_cartesian_->Forward(
    fix.latitude_deg,
    fix.longitude_deg,
    fix.altitude_m,
    east_m,
    north_m,
    up_m);
  return enuToMap(gtsam::Point3(east_m, north_m, up_m));
}

gtsam::Point3 FrameAnchor::enuToMap(const gtsam::Point3 & enu) const
{
  const auto rotated = map_R_enu_.rotate(enu);
  return gtsam::Point3(
    rotated.x() + map_t_enu_.x(),
    rotated.y() + map_t_enu_.y(),
    rotated.z() + map_t_enu_.z());
}

std::optional<double> FrameAnchor::enuYawToMapYaw(double enu_yaw_rad) const
{
  if (!initialized_ || !std::isfinite(enu_yaw_rad)) {
    return std::nullopt;
  }
  return normalizeYaw(map_R_enu_.yaw() + enu_yaw_rad);
}

std::string FrameAnchor::statusString() const
{
  if (!initialized_) {
    return "uninitialized";
  }
  std::ostringstream out;
  out << "initialized origin_lat=" << origin_.latitude_deg
      << " origin_lon=" << origin_.longitude_deg
      << " yaw_source=" << (yaw_from_heading_ ? "rtk_heading" : "identity");
  return out.str();
}

}  // namespace rtk_fgo_localizer
