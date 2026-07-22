#include "fgo_gil_localizer/livox_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fgo_gil_localizer
{

LivoxPreprocessor::LivoxPreprocessor(LivoxPreprocessorConfig config)
: config_(config)
{
  if (config_.point_stride == 0U || !std::isfinite(config_.minimum_range_m) ||
    !std::isfinite(config_.maximum_range_m) || config_.minimum_range_m < 0.0 ||
    config_.maximum_range_m <= config_.minimum_range_m ||
    config_.maximum_line_exclusive == 0U)
  {
    throw std::invalid_argument("Livox preprocessing configuration is outside valid bounds");
  }
}

std::vector<TimedLidarPoint> LivoxPreprocessor::process(
  const std::vector<RawLivoxPoint> & points,
  const std::size_t declared_point_count)
{
  ++diagnostics_.frames;
  if (declared_point_count != points.size()) {
    ++diagnostics_.declared_size_mismatches;
  }
  const std::size_t usable_count = std::min(declared_point_count, points.size());
  std::vector<TimedLidarPoint> output;
  output.reserve(usable_count / config_.point_stride + 1U);
  const double minimum_range_squared = config_.minimum_range_m * config_.minimum_range_m;
  const double maximum_range_squared = config_.maximum_range_m * config_.maximum_range_m;
  for (std::size_t index = 0; index < usable_count; index += config_.point_stride) {
    const auto & point = points[index];
    if (!finite(point.position)) {
      ++diagnostics_.rejected_nonfinite;
      continue;
    }
    if (point.line >= config_.maximum_line_exclusive) {
      ++diagnostics_.rejected_line;
      continue;
    }
    const std::uint8_t return_type = point.tag & 0x30U;
    if (return_type != 0x00U && return_type != 0x10U) {
      ++diagnostics_.rejected_tag;
      continue;
    }
    const double range_squared = squaredNorm(point.position);
    if (!std::isfinite(range_squared) || range_squared < minimum_range_squared ||
      range_squared > maximum_range_squared)
    {
      ++diagnostics_.rejected_range;
      continue;
    }
    output.push_back(
      {
        static_cast<double>(point.offset_time_ns) * 1.0e-9,
        point.position,
        static_cast<double>(point.reflectivity),
        point.line});
    ++diagnostics_.accepted;
  }
  std::stable_sort(
    output.begin(), output.end(), [](const TimedLidarPoint & left, const TimedLidarPoint & right) {
      return left.offset_s < right.offset_s;
    });
  return output;
}

}  // namespace fgo_gil_localizer
