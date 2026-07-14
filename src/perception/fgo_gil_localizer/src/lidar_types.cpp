#include "fgo_gil_localizer/lidar_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fgo_gil_localizer
{
namespace
{

Quaternion slerp(const Quaternion & first_input, const Quaternion & second_input, double ratio)
{
  Quaternion first = first_input.normalized();
  Quaternion second = second_input.normalized();
  double cosine = first.w * second.w + first.x * second.x + first.y * second.y +
    first.z * second.z;
  if (cosine < 0.0) {
    second = {-second.w, -second.x, -second.y, -second.z};
    cosine = -cosine;
  }
  cosine = std::clamp(cosine, -1.0, 1.0);
  if (cosine > 0.9995) {
    return Quaternion{
      first.w + ratio * (second.w - first.w),
      first.x + ratio * (second.x - first.x),
      first.y + ratio * (second.y - first.y),
      first.z + ratio * (second.z - first.z)}.normalized();
  }
  const double angle = std::acos(cosine);
  const double sine = std::sin(angle);
  if (!std::isfinite(sine) || std::abs(sine) <= std::numeric_limits<double>::epsilon()) {
    return first;
  }
  const double first_scale = std::sin((1.0 - ratio) * angle) / sine;
  const double second_scale = std::sin(ratio * angle) / sine;
  return Quaternion{
    first_scale * first.w + second_scale * second.w,
    first_scale * first.x + second_scale * second.x,
    first_scale * first.y + second_scale * second.y,
    first_scale * first.z + second_scale * second.z}.normalized();
}

}  // namespace

bool finite(const RigidPose & pose) noexcept
{
  return finite(pose.rotation) && finite(pose.translation);
}

Vec3 transformPoint(const RigidPose & pose, const Vec3 & point)
{
  return pose.rotation.rotate(point) + pose.translation;
}

RigidPose compose(const RigidPose & parent_child, const RigidPose & child_object)
{
  return {
    (parent_child.rotation * child_object.rotation).normalized(),
    transformPoint(parent_child, child_object.translation)};
}

RigidPose inverse(const RigidPose & pose)
{
  const Quaternion inverse_rotation = pose.rotation.normalized().conjugate();
  return {inverse_rotation, inverse_rotation.rotate(-pose.translation)};
}

RigidPose interpolatePose(const RigidPose & first, const RigidPose & second, const double ratio)
{
  if (!std::isfinite(ratio)) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {{invalid, invalid, invalid, invalid}, {invalid, invalid, invalid}};
  }
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  return {
    slerp(first.rotation, second.rotation, bounded_ratio),
    first.translation + bounded_ratio * (second.translation - first.translation)};
}

RigidPose leftPerturbPose(const RigidPose & pose, const std::array<double, 6> & delta)
{
  const RigidPose perturbation{
    quaternionFromRotationVector({delta[3], delta[4], delta[5]}),
    {delta[0], delta[1], delta[2]}};
  return compose(perturbation, pose);
}

double rotationDistanceRad(const Quaternion & first, const Quaternion & second)
{
  if (!finite(first) || !finite(second)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return norm(quaternionLog(first.normalized().conjugate() * second.normalized()));
}

}  // namespace fgo_gil_localizer
