#pragma once

#include <array>
#include <cstddef>
#include <cmath>

namespace fgo_gil_localizer
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double & operator[](std::size_t index) {return index == 0U ? x : (index == 1U ? y : z);}
  double operator[](std::size_t index) const {return index == 0U ? x : (index == 1U ? y : z);}
};

inline Vec3 operator+(const Vec3 & left, const Vec3 & right)
{
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 operator-(const Vec3 & left, const Vec3 & right)
{
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 operator-(const Vec3 & value)
{
  return {-value.x, -value.y, -value.z};
}

inline Vec3 operator*(const Vec3 & value, double scale)
{
  return {value.x * scale, value.y * scale, value.z * scale};
}

inline Vec3 operator*(double scale, const Vec3 & value)
{
  return value * scale;
}

inline Vec3 operator/(const Vec3 & value, double scale)
{
  return {value.x / scale, value.y / scale, value.z / scale};
}

inline double dot(const Vec3 & left, const Vec3 & right)
{
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline Vec3 cross(const Vec3 & left, const Vec3 & right)
{
  return {
    left.y * right.z - left.z * right.y,
    left.z * right.x - left.x * right.z,
    left.x * right.y - left.y * right.x};
}

inline double squaredNorm(const Vec3 & value)
{
  return dot(value, value);
}

inline double norm(const Vec3 & value)
{
  return std::sqrt(squaredNorm(value));
}

inline bool finite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Quaternion conjugate() const {return {w, -x, -y, -z};}
  Quaternion normalized() const;
  Vec3 rotate(const Vec3 & value) const;
};

Quaternion operator*(const Quaternion & left, const Quaternion & right);
Quaternion quaternionFromRotationVector(const Vec3 & rotation_vector);
Vec3 quaternionLog(const Quaternion & quaternion);
bool finite(const Quaternion & quaternion);

using BiasJacobian = std::array<double, 9U * 6U>;

inline double & jacobianAt(BiasJacobian & jacobian, std::size_t row, std::size_t column)
{
  return jacobian[row * 6U + column];
}

inline double jacobianAt(const BiasJacobian & jacobian, std::size_t row, std::size_t column)
{
  return jacobian[row * 6U + column];
}

}  // namespace fgo_gil_localizer
