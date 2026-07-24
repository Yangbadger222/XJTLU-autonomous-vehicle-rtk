#include <gtest/gtest.h>

#include "um982_rtk_driver/heading_selector.hpp"

using um982_rtk_driver::HeadingSelector;
using um982_rtk_driver::HeadingSelectorConfig;
using um982_rtk_driver::HeadingSource;

TEST(HeadingSelector, KeepsPrimarySourceWhileItIsFresh)
{
  HeadingSelector selector;

  EXPECT_TRUE(selector.observe(HeadingSource::THS, 10.0, 0.0).publish);
  EXPECT_FALSE(selector.observe(HeadingSource::UNIHEADING, 95.0, 0.1).publish);
  EXPECT_TRUE(selector.observe(HeadingSource::THS, 11.0, 0.2).publish);
  ASSERT_TRUE(selector.activeSource().has_value());
  EXPECT_EQ(*selector.activeSource(), HeadingSource::THS);
}

TEST(HeadingSelector, AlignsFallbackToLastTrustedHeadingBeforeSwitching)
{
  HeadingSelectorConfig config;
  config.fallback_timeout_s = 1.0;
  config.switch_min_samples = 2;
  HeadingSelector selector(config);

  EXPECT_TRUE(selector.observe(HeadingSource::THS, 350.0, 0.0).publish);
  EXPECT_FALSE(selector.observe(HeadingSource::UNIHEADING, 80.0, 1.1).publish);
  const auto switched = selector.observe(HeadingSource::UNIHEADING, 81.0, 1.2);

  EXPECT_TRUE(switched.publish);
  EXPECT_TRUE(switched.source_switched);
  EXPECT_NEAR(switched.heading_deg, 350.0, 1e-9);
  ASSERT_TRUE(selector.activeSource().has_value());
  EXPECT_EQ(*selector.activeSource(), HeadingSource::UNIHEADING);
}

TEST(HeadingSelector, RejectsNonPhysicalJumpFromActiveSource)
{
  HeadingSelector selector;

  EXPECT_TRUE(selector.observe(HeadingSource::THS, 20.0, 0.0).publish);
  const auto jump = selector.observe(HeadingSource::THS, 55.0, 0.1);
  const auto stable = selector.observe(HeadingSource::THS, 22.0, 0.2);

  EXPECT_FALSE(jump.publish);
  EXPECT_TRUE(jump.continuity_rejected);
  EXPECT_EQ(selector.rejectedCount(), 1);
  EXPECT_TRUE(stable.publish);
  EXPECT_NEAR(stable.heading_deg, 22.0, 1e-9);
}

TEST(HeadingSelector, UsesFallbackAfterPrimaryNeverAppears)
{
  HeadingSelectorConfig config;
  config.fallback_timeout_s = 1.0;
  HeadingSelector selector(config);

  EXPECT_FALSE(selector.observe(HeadingSource::UNIHEADING, 90.0, 0.0).publish);
  EXPECT_FALSE(selector.observe(HeadingSource::UNIHEADING, 90.0, 0.9).publish);
  const auto fallback = selector.observe(HeadingSource::UNIHEADING, 91.0, 1.1);

  EXPECT_TRUE(fallback.publish);
  EXPECT_NEAR(fallback.heading_deg, 91.0, 1e-9);
  ASSERT_TRUE(selector.activeSource().has_value());
  EXPECT_EQ(*selector.activeSource(), HeadingSource::UNIHEADING);
}
