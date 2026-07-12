// Copyright 2026 XJTLU Autonomous Vehicle Team

#include <gtest/gtest.h>

#include "pointcloud_to_laserscan/self_filter.hpp"

using pointcloud_to_laserscan::SelfFilterBox;

TEST(SelfFilterBox, DisabledBoxNeverRejectsPoints)
{
  const SelfFilterBox box{false, -0.35, 0.35, -0.275, 0.275};
  EXPECT_TRUE(box.valid());
  EXPECT_FALSE(box.contains(0.0, 0.0));
}

TEST(SelfFilterBox, IncludesVehicleEnvelopeAndBoundaries)
{
  const SelfFilterBox box{true, -0.35, 0.35, -0.275, 0.275};
  EXPECT_TRUE(box.valid());
  EXPECT_TRUE(box.contains(0.0, 0.0));
  EXPECT_TRUE(box.contains(-0.35, 0.275));
  EXPECT_FALSE(box.contains(0.351, 0.0));
  EXPECT_FALSE(box.contains(0.0, -0.276));
}

TEST(SelfFilterBox, RejectsInvertedBounds)
{
  EXPECT_FALSE((SelfFilterBox{true, 0.35, -0.35, -0.275, 0.275}).valid());
  EXPECT_FALSE((SelfFilterBox{true, -0.35, 0.35, 0.275, -0.275}).valid());
}
