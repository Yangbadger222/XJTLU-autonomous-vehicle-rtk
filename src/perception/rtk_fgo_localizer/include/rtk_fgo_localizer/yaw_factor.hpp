#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>

namespace rtk_fgo_localizer
{

class YawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  YawFactor(
    gtsam::Key pose_key,
    double measured_yaw_rad,
    const gtsam::SharedNoiseModel & model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override;

private:
  double measured_yaw_rad_ = 0.0;
};

}  // namespace rtk_fgo_localizer
