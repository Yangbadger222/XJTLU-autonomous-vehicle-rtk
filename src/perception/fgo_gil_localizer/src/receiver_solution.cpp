#include "fgo_gil_localizer/receiver_solution.hpp"

#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Cholesky>

namespace fgo_gil_localizer
{
namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;

std::optional<int> parseInteger(const std::string & value)
{
  try {
    std::size_t parsed = 0U;
    const int output = std::stoi(value, &parsed);
    return parsed == value.size() ? std::optional<int>{output} : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> parseFiniteDouble(const std::string & value)
{
  try {
    std::size_t parsed = 0U;
    const double output = std::stod(value, &parsed);
    if (parsed != value.size() || !std::isfinite(output)) {
      return std::nullopt;
    }
    return output;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool isGgaType(const std::string & value)
{
  return value.size() >= 3U && value.compare(value.size() - 3U, 3U, "GGA") == 0;
}

}  // namespace

std::optional<ReceiverRtkQuality> parseGgaRtkQuality(const std::string & sentence)
{
  const std::size_t start = sentence.find('$');
  if (start == std::string::npos) {
    return std::nullopt;
  }
  const std::size_t checksum = sentence.find('*', start);
  const std::string payload = sentence.substr(
    start + 1U,
    checksum == std::string::npos ? std::string::npos : checksum - start - 1U);
  std::stringstream stream(payload);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (fields.size() <= 8U || !isGgaType(fields.front())) {
    return std::nullopt;
  }
  const auto quality = parseInteger(fields[6]);
  const auto satellites = parseInteger(fields[7]);
  const auto hdop = parseFiniteDouble(fields[8]);
  if (!quality.has_value() || !satellites.has_value() || !hdop.has_value() ||
    *quality < 0 || *satellites < 0 || *hdop < 0.0)
  {
    return std::nullopt;
  }
  return ReceiverRtkQuality{*quality, *satellites, *hdop};
}

bool isAcceptedFixedRtk(
  const ReceiverRtkQuality & quality,
  const int fixed_quality_code,
  const int minimum_satellites,
  const double maximum_hdop)
{
  return quality.quality == fixed_quality_code && quality.satellites >= minimum_satellites &&
         std::isfinite(quality.hdop) && quality.hdop <= maximum_hdop;
}

std::optional<Vec3> geodeticToEcef(
  const double latitude_deg,
  const double longitude_deg,
  const double ellipsoid_altitude_m)
{
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
    !std::isfinite(ellipsoid_altitude_m) || std::abs(latitude_deg) > 90.0 ||
    std::abs(longitude_deg) > 180.0)
  {
    return std::nullopt;
  }
  const double latitude = latitude_deg * kDegreesToRadians;
  const double longitude = longitude_deg * kDegreesToRadians;
  const double sine_latitude = std::sin(latitude);
  const double cosine_latitude = std::cos(latitude);
  const double sine_longitude = std::sin(longitude);
  const double cosine_longitude = std::cos(longitude);
  const double eccentricity_squared = kWgs84Flattening * (2.0 - kWgs84Flattening);
  const double radius = kWgs84SemiMajorAxisM / std::sqrt(
    1.0 - eccentricity_squared * sine_latitude * sine_latitude);
  const Vec3 output{
    (radius + ellipsoid_altitude_m) * cosine_latitude * cosine_longitude,
    (radius + ellipsoid_altitude_m) * cosine_latitude * sine_longitude,
    (radius * (1.0 - eccentricity_squared) + ellipsoid_altitude_m) * sine_latitude};
  return finite(output) ? std::optional<Vec3>{output} : std::nullopt;
}

std::optional<Eigen::Matrix3d> enuCovarianceToEcef(
  const double latitude_deg,
  const double longitude_deg,
  const Eigen::Matrix3d & covariance_enu_m2)
{
  if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
    std::abs(latitude_deg) > 90.0 || std::abs(longitude_deg) > 180.0 ||
    !covariance_enu_m2.allFinite())
  {
    return std::nullopt;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (covariance_enu_m2 + covariance_enu_m2.transpose());
  Eigen::LLT<Eigen::Matrix3d> covariance_solver(symmetric);
  if (covariance_solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  const double latitude = latitude_deg * kDegreesToRadians;
  const double longitude = longitude_deg * kDegreesToRadians;
  const double sine_latitude = std::sin(latitude);
  const double cosine_latitude = std::cos(latitude);
  const double sine_longitude = std::sin(longitude);
  const double cosine_longitude = std::cos(longitude);
  Eigen::Matrix3d ecef_from_enu;
  ecef_from_enu <<
    -sine_longitude, -sine_latitude * cosine_longitude, cosine_latitude * cosine_longitude,
    cosine_longitude, -sine_latitude * sine_longitude, cosine_latitude * sine_longitude,
    0.0, cosine_latitude, sine_latitude;
  const Eigen::Matrix3d output = ecef_from_enu * symmetric * ecef_from_enu.transpose();
  return output.allFinite() ? std::optional<Eigen::Matrix3d>{output} : std::nullopt;
}

}  // namespace fgo_gil_localizer
