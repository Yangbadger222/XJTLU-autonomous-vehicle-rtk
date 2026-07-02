#include <gtest/gtest.h>

#include "rtk_fgo_localizer/gate_params.hpp"
#include "rtk_fgo_localizer/rtk_quality.hpp"

namespace rtk_fgo_localizer
{
namespace
{

TEST(RtkGateParams, CustomFixedQualityCodeIsUsed)
{
  RtkGateParams params;
  params.fixed_quality_code = 9;
  params.min_satellites = 6;
  params.max_hdop = 3.0;

  RtkGateInput input;
  input.quality.quality = 9;
  input.quality.satellites = 8;
  input.quality.hdop = 1.5;
  input.position_innovation_m = 0.2;
  input.heading_innovation_rad = 0.1;

  auto decision = evaluateRtkGate(input, params);

  EXPECT_EQ(decision.mode, RtkGateMode::StrongCandidate);
}

TEST(RtkGateParams, CustomMaxPositionInnovationRejects)
{
  RtkGateParams params;
  params.max_strong_position_innovation_m = 0.5;

  RtkGateInput input;
  input.quality.quality = 4;
  input.quality.satellites = 18;
  input.quality.hdop = 0.7;
  input.position_innovation_m = 0.7;
  input.heading_innovation_rad = 0.1;

  auto decision = evaluateRtkGate(input, params);

  EXPECT_EQ(decision.mode, RtkGateMode::Rejected);
}

TEST(RtkGateParams, ImpliedSpeedRejectsStrongCandidate)
{
  RtkGateParams params;
  params.max_implied_speed_mps = 1.0;

  RtkGateInput input;
  input.quality.quality = 4;
  input.quality.satellites = 18;
  input.quality.hdop = 0.7;
  input.position_innovation_m = 0.2;
  input.heading_innovation_rad = 0.1;
  input.implied_speed_mps = 3.0;

  auto decision = evaluateRtkGate(input, params);

  EXPECT_EQ(decision.mode, RtkGateMode::Rejected);
}

TEST(RtkGateParams, MissingSpeedHistoryDoesNotReject)
{
  RtkGateParams params;
  params.max_implied_speed_mps = 1.0;

  RtkGateInput input;
  input.quality.quality = 4;
  input.quality.satellites = 18;
  input.quality.hdop = 0.7;
  input.position_innovation_m = 0.2;
  input.heading_innovation_rad = 0.1;

  auto decision = evaluateRtkGate(input, params);

  EXPECT_EQ(decision.mode, RtkGateMode::StrongCandidate);
}

}  // namespace
}  // namespace rtk_fgo_localizer
