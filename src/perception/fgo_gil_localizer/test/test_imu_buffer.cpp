#include <gtest/gtest.h>

#include <limits>

#include "fgo_gil_localizer/imu_buffer.hpp"

namespace fgo_gil_localizer
{
namespace
{

ImuSample sample(double stamp_s)
{
  return {stamp_s, {0.0, 0.0, 9.8}, {0.0, 0.0, 0.0}};
}

TEST(ImuSegmentBuffer, BoundsStorageAndRejectsDuplicate)
{
  ImuSegmentBuffer buffer({3, 0.05});
  EXPECT_EQ(buffer.add(sample(0.00)), ImuBufferResult::Accepted);
  EXPECT_EQ(buffer.add(sample(0.01)), ImuBufferResult::Accepted);
  EXPECT_EQ(buffer.add(sample(0.01)), ImuBufferResult::RejectedDuplicate);
  EXPECT_EQ(buffer.add(sample(0.02)), ImuBufferResult::Accepted);
  EXPECT_EQ(buffer.add(sample(0.03)), ImuBufferResult::AcceptedWithEviction);
  EXPECT_EQ(buffer.size(), 3U);
  EXPECT_NEAR(buffer.samples().front().stamp_s, 0.01, 1.0e-12);
}

TEST(ImuSegmentBuffer, StartsNewSegmentOnGapAndTimeReversal)
{
  ImuSegmentBuffer buffer({16, 0.05});
  EXPECT_EQ(buffer.add(sample(1.0)), ImuBufferResult::Accepted);
  EXPECT_EQ(buffer.add(sample(1.2)), ImuBufferResult::ResetOnGap);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_NEAR(buffer.samples().front().stamp_s, 1.2, 1.0e-12);
  EXPECT_EQ(buffer.add(sample(0.5)), ImuBufferResult::ResetOnTimeReversal);
  EXPECT_EQ(buffer.diagnostics().segment_id, 2U);
}

TEST(ImuSegmentBuffer, RejectsNonFiniteWithoutPoisoningSegment)
{
  ImuSegmentBuffer buffer;
  EXPECT_EQ(buffer.add(sample(1.0)), ImuBufferResult::Accepted);
  auto invalid = sample(1.01);
  invalid.acceleration_m_s2.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(buffer.add(invalid), ImuBufferResult::RejectedNonFinite);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.add(sample(1.01)), ImuBufferResult::Accepted);
}

}  // namespace
}  // namespace fgo_gil_localizer
