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
