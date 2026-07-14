#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "fgo_gil_localizer/ecef_imu_preintegrator.hpp"

namespace fgo_gil_localizer
{
namespace
{

constexpr double kEarthRadiusM = 6378137.0;

EcefState initialState()
{
  EcefState state;
  state.position_ecef_m = {kEarthRadiusM, 0.0, 0.0};
  return state;
}

void integrateConstant(
  EcefImuPreintegrator & integrator,
  const Vec3 & acceleration,
  const Vec3 & angular_velocity,
  double duration_s,
  double step_s)
{
  ASSERT_EQ(
    integrator.integrate({0.0, acceleration, angular_velocity}),
    ImuIntegrationResult::Initialized);
  const int steps = static_cast<int>(std::llround(duration_s / step_s));
  for (int index = 1; index <= steps; ++index) {
    ASSERT_EQ(
      integrator.integrate({index * step_s, acceleration, angular_velocity}),
      ImuIntegrationResult::Integrated);
  }
}

TEST(EcefImuPreintegrator, PropagatesConstantAccelerationWithoutEarthTerms)
{
  EcefImuConfig config;
  config.earth_rotation_rad_s = 0.0;
  config.gravitational_parameter_m3_s2 = 0.0;
  EcefImuPreintegrator integrator(config);
  ASSERT_TRUE(integrator.reset(initialState()));
  integrateConstant(integrator, {1.0, 0.0, 0.0}, {}, 1.0, 0.01);

  EXPECT_NEAR(integrator.state().position_ecef_m.x, kEarthRadiusM + 0.5, 1.0e-8);
  EXPECT_NEAR(integrator.state().velocity_ecef_m_s.x, 1.0, 1.0e-10);
  EXPECT_NEAR(integrator.diagnostics().integrated_duration_s, 1.0, 1.0e-12);
}

TEST(EcefImuPreintegrator, KeepsStationaryEquatorStateWithEarthRotation)
{
  EcefImuConfig config;
  EcefImuPreintegrator integrator(config);
  ASSERT_TRUE(integrator.reset(initialState()));
  const double gravitational_acceleration =
    config.gravitational_parameter_m3_s2 / (kEarthRadiusM * kEarthRadiusM);
  const double centrifugal_acceleration =
    config.earth_rotation_rad_s * config.earth_rotation_rad_s * kEarthRadiusM;
  const Vec3 stationary_specific_force{
    gravitational_acceleration - centrifugal_acceleration, 0.0, 0.0};
  const Vec3 stationary_gyro{0.0, 0.0, config.earth_rotation_rad_s};
  integrateConstant(integrator, stationary_specific_force, stationary_gyro, 10.0, 0.01);

  EXPECT_NEAR(integrator.state().position_ecef_m.x, kEarthRadiusM, 1.0e-5);
  EXPECT_NEAR(norm(integrator.state().velocity_ecef_m_s), 0.0, 1.0e-6);
  EXPECT_NEAR(integrator.state().orientation_ecef_body.w, 1.0, 1.0e-10);
}

TEST(EcefImuPreintegrator, ProducesBiasSensitivityWithExpectedSigns)
{
  EcefImuConfig config;
  config.earth_rotation_rad_s = 0.0;
  config.gravitational_parameter_m3_s2 = 0.0;
  EcefImuPreintegrator integrator(config);
  ASSERT_TRUE(integrator.reset(initialState()));
  integrateConstant(integrator, {1.0, 0.0, 0.0}, {}, 1.0, 0.01);

  const auto jacobian = integrator.biasJacobian();
  EXPECT_NEAR(jacobianAt(jacobian, 0, 0), -0.5, 2.0e-4);
  EXPECT_NEAR(jacobianAt(jacobian, 3, 0), -1.0, 2.0e-4);
  EXPECT_NEAR(jacobianAt(jacobian, 8, 5), -1.0, 2.0e-4);
}

TEST(EcefImuPreintegrator, FailsClosedOnBadTimestampsAndMeasurements)
{
  EcefImuPreintegrator integrator;
  EXPECT_EQ(integrator.integrate({}), ImuIntegrationResult::RejectedNotReset);
  ASSERT_TRUE(integrator.reset(initialState()));
  EXPECT_EQ(integrator.integrate({0.0, {}, {}}), ImuIntegrationResult::Initialized);
  EXPECT_EQ(integrator.integrate({0.0, {}, {}}), ImuIntegrationResult::RejectedDuplicate);
  EXPECT_EQ(integrator.integrate({0.1, {}, {}}), ImuIntegrationResult::RejectedGap);
  EXPECT_FALSE(integrator.valid());

  ASSERT_TRUE(integrator.reset(initialState()));
  auto invalid = ImuSample{0.0, {}, {}};
  invalid.angular_velocity_rad_s.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(integrator.integrate(invalid), ImuIntegrationResult::RejectedNonFinite);
  EXPECT_FALSE(integrator.valid());
}

}  // namespace
}  // namespace fgo_gil_localizer
