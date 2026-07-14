#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "fgo_gil_localizer/lidar_types.hpp"

namespace fgo_gil_localizer
{

struct LivoxPreprocessorConfig
{
  std::size_t point_stride = 1;
  double minimum_range_m = 0.5;
  double maximum_range_m = 25.0;
  std::uint8_t maximum_line_exclusive = 4;
};

struct LivoxPreprocessorDiagnostics
{
  std::uint64_t frames = 0;
  std::uint64_t declared_size_mismatches = 0;
  std::uint64_t accepted = 0;
  std::uint64_t rejected_line = 0;
  std::uint64_t rejected_tag = 0;
  std::uint64_t rejected_range = 0;
  std::uint64_t rejected_nonfinite = 0;
};

class LivoxPreprocessor
{
public:
  explicit LivoxPreprocessor(LivoxPreprocessorConfig config = {});

  std::vector<TimedLidarPoint> process(
    const std::vector<RawLivoxPoint> & points,
    std::size_t declared_point_count);

  const LivoxPreprocessorDiagnostics & diagnostics() const noexcept {return diagnostics_;}

private:
  LivoxPreprocessorConfig config_;
  LivoxPreprocessorDiagnostics diagnostics_;
};

}  // namespace fgo_gil_localizer
