#include "um982_rtk_driver/ntrip_reconnect_policy.hpp"

#include <gtest/gtest.h>

namespace um982_rtk_driver
{
namespace
{

TEST(NtripReconnectPolicyTest, ReusesFreshCachedValidGgaBeforeConnectingWithoutGga)
{
  NtripReconnectPolicy policy(NtripReconnectConfig{});

  const auto gga = policy.initialGga("$GPGGA,invalid", false, "$GPGGA,fixed", 29.9);

  ASSERT_TRUE(gga.has_value());
  EXPECT_EQ(*gga, "$GPGGA,fixed");
}

TEST(NtripReconnectPolicyTest, ConnectsWithoutGgaWhenNoFreshValidSampleExists)
{
  NtripReconnectPolicy policy(NtripReconnectConfig{});

  const auto gga = policy.initialGga("$GPGGA,no_fix", false, "$GPGGA,old_fixed", 30.1);

  ASSERT_TRUE(gga.has_value());
  EXPECT_TRUE(gga->empty());
}

TEST(NtripReconnectPolicyTest, CanRetainLegacyNoGgaBlockWhenExplicitlyDisabled)
{
  NtripReconnectConfig config;
  config.reconnect_without_valid_gga = false;
  NtripReconnectPolicy policy(config);

  EXPECT_FALSE(policy.initialGga("", false, "", 0.0).has_value());
}

TEST(NtripReconnectPolicyTest, CapsExponentialBackoffAndBoundsJitter)
{
  NtripReconnectConfig config;
  config.initial_backoff_s = 1.0;
  config.max_backoff_s = 10.0;
  config.jitter_s = 0.2;
  NtripReconnectPolicy policy(config);

  EXPECT_EQ(policy.retryDelay(1, 0.0).count(), 1000);
  EXPECT_EQ(policy.retryDelay(4, 0.0).count(), 8000);
  EXPECT_EQ(policy.retryDelay(8, 1.0).count(), 10200);
}

}  // namespace
}  // namespace um982_rtk_driver
