#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "fgo_gil_localizer/lidar_factors.hpp"

namespace fgo_gil_localizer
{
namespace
{

TEST(LidarFactors, HaveCorrectZeroResiduals)
{
  const auto plane = evaluatePointToPlane(
    {{1.0, 2.0, 0.0}, {}, {0.0, 0.0, 1.0}}, RigidPose{});
  const auto line = evaluatePointToLine(
    {{2.0, 0.0, 0.0}, {}, {1.0, 0.0, 0.0}}, RigidPose{});

  ASSERT_TRUE(plane.has_value());
  ASSERT_TRUE(line.has_value());
  EXPECT_NEAR(plane->residual, 0.0, 1.0e-12);
  EXPECT_NEAR(line->residual[0], 0.0, 1.0e-12);
  EXPECT_NEAR(line->residual[1], 0.0, 1.0e-12);
}

TEST(LidarFactors, AnalyticJacobiansMatchLeftPerturbationFiniteDifferences)
{
  const RigidPose pose{
    quaternionFromRotationVector({0.1, -0.2, 0.05}),
    {1.0, -2.0, 0.5}};
  const PointToPlaneFactor plane{{2.0, 0.5, -0.3}, {0.0, 0.0, 1.0}, {0.2, -0.4, 1.0}};
  const PointToLineFactor line{{1.0, -0.5, 0.2}, {0.2, 1.0, -0.3}, {1.0, 2.0, 0.5}};
  const auto plane_evaluation = evaluatePointToPlane(plane, pose);
  const auto line_evaluation = evaluatePointToLine(line, pose);
  ASSERT_TRUE(plane_evaluation.has_value());
  ASSERT_TRUE(line_evaluation.has_value());

  constexpr double epsilon = 1.0e-7;
  for (std::size_t column = 0; column < 6U; ++column) {
    std::array<double, 6> positive{};
    std::array<double, 6> negative{};
    positive[column] = epsilon;
    negative[column] = -epsilon;
    const auto plane_positive = evaluatePointToPlane(plane, leftPerturbPose(pose, positive));
    const auto plane_negative = evaluatePointToPlane(plane, leftPerturbPose(pose, negative));
    const auto line_positive = evaluatePointToLine(line, leftPerturbPose(pose, positive));
    const auto line_negative = evaluatePointToLine(line, leftPerturbPose(pose, negative));
    ASSERT_TRUE(plane_positive.has_value());
    ASSERT_TRUE(plane_negative.has_value());
    ASSERT_TRUE(line_positive.has_value());
    ASSERT_TRUE(line_negative.has_value());
    EXPECT_NEAR(
      plane_evaluation->jacobian[column],
      (plane_positive->residual - plane_negative->residual) / (2.0 * epsilon), 1.0e-7);
    for (std::size_t row = 0; row < 2U; ++row) {
      EXPECT_NEAR(
        line_evaluation->jacobian[row][column],
        (line_positive->residual[row] - line_negative->residual[row]) / (2.0 * epsilon),
        1.0e-7);
    }
  }
}

TEST(LidarFactors, FullRankGeometryCanProduceAValidConstraint)
{
  std::vector<PointToPlaneFactor> planes{
    {{0.0, 0.0, 0.0}, {}, {1.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}, {}, {0.0, 1.0, 0.0}},
    {{0.0, 0.0, 0.0}, {}, {0.0, 0.0, 1.0}},
    {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
    {{0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}};
  std::vector<PointToLineFactor> lines{
    {{1.0, 0.0, 0.0}, {}, {1.0, 0.0, 0.0}}};
  LidarConstraintConfig config;
  config.minimum_line_matches = 1U;
  config.minimum_plane_matches = planes.size();
  config.minimum_information_eigenvalue = 1.0e-6;

  const auto summary = analyzeLidarFactors(lines, planes, RigidPose{}, config);

  EXPECT_FALSE(summary.degenerate);
  EXPECT_TRUE(summary.constraint_valid);
  EXPECT_NEAR(summary.residual_rms_m, 0.0, 1.0e-12);
}

}  // namespace
}  // namespace fgo_gil_localizer
