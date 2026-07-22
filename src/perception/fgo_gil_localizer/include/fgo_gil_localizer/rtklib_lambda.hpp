#pragma once

#include <Eigen/Core>

namespace fgo_gil_localizer
{
namespace rtklib
{

struct LambdaResult
{
  Eigen::MatrixXd candidates;
  Eigen::VectorXd squared_norms;
  Eigen::VectorXd conditional_variances;
};

// RTKLIB MLAMBDA uses column-major covariance and candidate storage. Eigen's
// default dense storage has the same convention, while this wrapper keeps the
// ownership and validation on the C++ side.
bool lambda(
  const Eigen::VectorXd & float_ambiguities,
  const Eigen::MatrixXd & covariance,
  int candidate_count,
  LambdaResult & result);

}  // namespace rtklib
}  // namespace fgo_gil_localizer
