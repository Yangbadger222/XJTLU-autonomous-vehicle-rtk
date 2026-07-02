#include <gtest/gtest.h>

#include "rtk_fgo_localizer/topic_buffers.hpp"

namespace rtk_fgo_localizer
{
namespace
{

TEST(TopicBuffers, PrunesSamplesOutsideMaxAge)
{
  TimestampedBuffer<int> buffer(1.0);
  buffer.add(0.0, 1);
  buffer.add(0.5, 2);
  buffer.add(1.2, 3);

  EXPECT_EQ(buffer.size(), 2u);
  ASSERT_TRUE(buffer.latest().has_value());
  EXPECT_EQ(buffer.latest()->value, 3);
}

TEST(TopicBuffers, ReturnsClosestWithinTolerance)
{
  TimestampedBuffer<int> buffer(5.0);
  buffer.add(1.0, 10);
  buffer.add(2.0, 20);
  buffer.add(3.0, 30);

  auto sample = buffer.closest(2.2, 0.25);

  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->value, 20);
}

TEST(TopicBuffers, ReturnsNoClosestOutsideTolerance)
{
  TimestampedBuffer<int> buffer(5.0);
  buffer.add(1.0, 10);

  auto sample = buffer.closest(2.0, 0.25);

  EXPECT_FALSE(sample.has_value());
}

}  // namespace
}  // namespace rtk_fgo_localizer
