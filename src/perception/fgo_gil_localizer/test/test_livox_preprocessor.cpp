#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "fgo_gil_localizer/livox_preprocessor.hpp"

namespace fgo_gil_localizer
{
namespace
{

RawLivoxPoint point(const double x, const std::uint8_t line = 0U, const std::uint8_t tag = 0U)
{
  RawLivoxPoint output;
  output.position = {x, 0.0, 0.0};
  output.line = line;
  output.tag = tag;
  return output;
}

TEST(LivoxPreprocessor, AppliesPublicMid360ValidityRules)
{
  LivoxPreprocessor preprocessor;
  auto invalid_nonfinite = point(2.0);
  invalid_nonfinite.position.z = std::numeric_limits<double>::quiet_NaN();
  const std::vector<RawLivoxPoint> input{
    point(2.0), point(3.0, 1U, 0x10U), point(0.1), point(2.0, 4U),
    point(2.0, 0U, 0x20U), invalid_nonfinite};

  const auto output = preprocessor.process(input, input.size() + 1U);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_DOUBLE_EQ(output[0].position.x, 2.0);
  EXPECT_DOUBLE_EQ(output[1].position.x, 3.0);
  EXPECT_EQ(preprocessor.diagnostics().declared_size_mismatches, 1U);
  EXPECT_EQ(preprocessor.diagnostics().rejected_range, 1U);
  EXPECT_EQ(preprocessor.diagnostics().rejected_line, 1U);
  EXPECT_EQ(preprocessor.diagnostics().rejected_tag, 1U);
  EXPECT_EQ(preprocessor.diagnostics().rejected_nonfinite, 1U);
}

TEST(LivoxPreprocessor, SortsAcceptedPointsByPerPointOffset)
{
  LivoxPreprocessor preprocessor;
  auto later = point(2.0);
  later.offset_time_ns = 20000000U;
  auto earlier = point(2.0);
  earlier.offset_time_ns = 10000000U;

  const auto output = preprocessor.process({later, earlier}, 2U);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_NEAR(output[0].offset_s, 0.01, 1.0e-12);
  EXPECT_NEAR(output[1].offset_s, 0.02, 1.0e-12);
}

}  // namespace
}  // namespace fgo_gil_localizer
