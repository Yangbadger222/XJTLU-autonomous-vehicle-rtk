#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace frc_nodes_cpp
{

constexpr uint8_t kSourceMapAnchor = 1;
constexpr uint8_t kSourceFeatureRetrieval = 2;
constexpr uint8_t kSourceModel = 4;

struct RiskPipelineConfig
{
  double size_m{12.0};
  double resolution{0.1};
  double agg_window_s{1.5};
  double z_min{-0.35};
  double z_max{0.32};

  int gridSize() const
  {
    return static_cast<int>(std::lround(size_m / resolution));
  }
};

struct AnchorStateLite
{
  std::string state{"candidate"};
  double p_usable{0.0};
  double severity{0.0};
  double map_x{0.0};
  double map_y{0.0};
};

struct RiskFrame
{
  RiskFrame(const int grid_size, const double grid_resolution,
            const double origin_x_in, const double origin_y_in)
  : n(grid_size),
    resolution(grid_resolution),
    origin_x(origin_x_in),
    origin_y(origin_y_in),
    risk(static_cast<std::size_t>(grid_size * grid_size), 0.0F),
    confidence(static_cast<std::size_t>(grid_size * grid_size), 0.0F),
    source(static_cast<std::size_t>(grid_size * grid_size), 0U)
  {
  }

  std::size_t index(const int row, const int col) const
  {
    return static_cast<std::size_t>(row * n + col);
  }

  int n;
  double resolution;
  double origin_x;
  double origin_y;
  std::vector<float> risk;
  std::vector<float> confidence;
  std::vector<uint8_t> source;
};

inline double injectionWeight(const AnchorStateLite & anchor)
{
  if (anchor.state == "retired") {
    return 0.0;
  }
  double weight = anchor.severity * anchor.p_usable;
  if (anchor.state == "stale") {
    weight *= 0.5;
  }
  return std::clamp(weight, 0.0, 1.0);
}

inline void renderGaussian(
  RiskFrame & frame,
  const double cx,
  const double cy,
  const float weight,
  const float sigma,
  const float confidence,
  const uint8_t source_bit)
{
  if (sigma <= 0.0F || weight <= 0.0F) {
    return;
  }

  const int radius_cells = std::max(
    static_cast<int>(3.0F * sigma / static_cast<float>(frame.resolution)), 1);
  const int ix = static_cast<int>(std::floor((cx - frame.origin_x) / frame.resolution));
  const int iy = static_cast<int>(std::floor((cy - frame.origin_y) / frame.resolution));
  const int x0 = std::max(ix - radius_cells, 0);
  const int x1 = std::min(ix + radius_cells + 1, frame.n);
  const int y0 = std::max(iy - radius_cells, 0);
  const int y1 = std::min(iy + radius_cells + 1, frame.n);
  if (x0 >= x1 || y0 >= y1) {
    return;
  }

  const float denom = 2.0F * sigma * sigma;
  for (int y = y0; y < y1; ++y) {
    const double wy = frame.origin_y + (static_cast<double>(y) + 0.5) * frame.resolution;
    for (int x = x0; x < x1; ++x) {
      const double wx = frame.origin_x + (static_cast<double>(x) + 0.5) * frame.resolution;
      const auto idx = frame.index(y, x);
      const double dx = wx - cx;
      const double dy = wy - cy;
      const float value = weight * std::exp(
        -static_cast<float>((dx * dx + dy * dy) / denom));
      if (value > frame.risk[idx]) {
        frame.risk[idx] = value;
        frame.confidence[idx] = confidence;
        frame.source[idx] = source_bit;
      }
    }
  }
}

inline std::vector<int8_t> toOccupancyData(const std::vector<float> & values01)
{
  std::vector<int8_t> out;
  out.reserve(values01.size());
  for (const float value : values01) {
    const int scaled = static_cast<int>(std::lround(
      std::clamp(value, 0.0F, 1.0F) * 100.0F));
    out.push_back(static_cast<int8_t>(std::clamp(scaled, 0, 100)));
  }
  return out;
}

}  // namespace frc_nodes_cpp
