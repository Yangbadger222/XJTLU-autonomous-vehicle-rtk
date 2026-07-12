#include <gtest/gtest.h>

#include "serial_twistctl/twist_command.hpp"

TEST(TwistCommand, FormatsAngularVelocityWithConfiguredScale) {
  EXPECT_EQ(
      serial_twistctl::formatTwistCommand(0.5, 0.2, -1.0),
      "vcx=0.500,wc=-0.200\n");
}

TEST(TwistCommand, KeepsAngularVelocityByDefault) {
  EXPECT_EQ(
      serial_twistctl::formatTwistCommand(0.5, 0.2, 1.0),
      "vcx=0.500,wc=0.200\n");
}

TEST(TwistCommand, LimitsOnlyMagnitudeIncrease) {
  EXPECT_DOUBLE_EQ(
      serial_twistctl::limitCommandAcceleration(0.0, 0.45, 0.03), 0.03);
  EXPECT_NEAR(
      serial_twistctl::limitCommandAcceleration(0.20, 0.45, 0.03), 0.23, 1e-12);
  EXPECT_DOUBLE_EQ(
      serial_twistctl::limitCommandAcceleration(0.45, 0.10, 0.03), 0.10);
  EXPECT_DOUBLE_EQ(
      serial_twistctl::limitCommandAcceleration(0.45, 0.0, 0.03), 0.0);
}

TEST(TwistCommand, PassesThroughZeroBeforeDirectionChange) {
  EXPECT_DOUBLE_EQ(
      serial_twistctl::limitCommandAcceleration(0.20, -0.20, 0.03), 0.0);
  EXPECT_DOUBLE_EQ(
      serial_twistctl::limitCommandAcceleration(0.0, -0.20, 0.03), -0.03);
}
