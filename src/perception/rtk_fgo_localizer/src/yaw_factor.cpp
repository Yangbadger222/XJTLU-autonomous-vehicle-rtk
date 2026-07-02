#include "rtk_fgo_localizer/yaw_factor.hpp"

#include "rtk_fgo_localizer/correction_smoother.hpp"

namespace rtk_fgo_localizer
{

YawFactor::YawFactor(
  gtsam::Key pose_key,
  double measured_yaw_rad,
  const gtsam::SharedNoiseModel & model)
: gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
  measured_yaw_rad_(measured_yaw_rad)
{
}

gtsam::Vector YawFactor::evaluateError(
  const gtsam::Pose3 & pose,
  boost::optional<gtsam::Matrix &> H) const
{
  if (H) {
    *H = gtsam::Matrix::Zero(1, 6);
    (*H)(0, 2) = 1.0;
  }
  return (gtsam::Vector(1) << normalizeYaw(pose.rotation().yaw() - measured_yaw_rad_)).finished();
}

}  // namespace rtk_fgo_localizer
