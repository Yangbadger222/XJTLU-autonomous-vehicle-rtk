#include "fgo_gil_localizer/math_types.hpp"

#include <algorithm>
#include <limits>

namespace fgo_gil_localizer
{

Quaternion Quaternion::normalized() const
{
  const double magnitude = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(magnitude) || magnitude <= std::numeric_limits<double>::epsilon()) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  return {w / magnitude, x / magnitude, y / magnitude, z / magnitude};
}

Vec3 Quaternion::rotate(const Vec3 & value) const
{
  const Quaternion unit = normalized();
  const Vec3 vector_part{unit.x, unit.y, unit.z};
  const Vec3 twice_cross = 2.0 * cross(vector_part, value);
  return value + unit.w * twice_cross + cross(vector_part, twice_cross);
}

Quaternion operator*(const Quaternion & left, const Quaternion & right)
{
  return {
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

Quaternion quaternionFromRotationVector(const Vec3 & rotation_vector)
{
  const double angle = norm(rotation_vector);
  if (!std::isfinite(angle)) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  if (angle < 1.0e-12) {
    return Quaternion{1.0, 0.5 * rotation_vector.x, 0.5 * rotation_vector.y,
      0.5 * rotation_vector.z}.normalized();
  }
  const double half_angle = 0.5 * angle;
  const double scale = std::sin(half_angle) / angle;
  return {
    std::cos(half_angle),
    rotation_vector.x * scale,
    rotation_vector.y * scale,
    rotation_vector.z * scale};
}

Vec3 quaternionLog(const Quaternion & quaternion)
{
  Quaternion unit = quaternion.normalized();
  if (unit.w < 0.0) {
    unit = {-unit.w, -unit.x, -unit.y, -unit.z};
  }
  const Vec3 vector_part{unit.x, unit.y, unit.z};
  const double vector_norm = norm(vector_part);
  if (vector_norm < 1.0e-12) {
    return 2.0 * vector_part;
  }
  const double angle = 2.0 * std::atan2(vector_norm, std::clamp(unit.w, -1.0, 1.0));
  return vector_part * (angle / vector_norm);
}

bool finite(const Quaternion & quaternion)
{
  return std::isfinite(quaternion.w) && std::isfinite(quaternion.x) &&
         std::isfinite(quaternion.y) && std::isfinite(quaternion.z);
}

}  // namespace fgo_gil_localizer
