/*
 * Integer least-squares reduction and search adapted from RTKLIB lambda.c.
 * Upstream: https://github.com/tomojitakasu/RTKLIB/blob/
 *           71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/lambda.c
 * Copyright (c) 2007-2013, T. Takasu, All rights reserved.
 *
 * The algorithm is unchanged. Allocation and the final linear solve use Eigen
 * so fgo_gil_localizer does not need the rest of RTKLIB. See
 * third_party/rtklib/LICENSE.txt for redistribution terms.
 */

#include "fgo_gil_localizer/rtklib_lambda.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace fgo_gil_localizer
{
namespace rtklib
{
namespace
{

constexpr int kLoopMaximum = 10000;

double sign(const double value)
{
  return value <= 0.0 ? -1.0 : 1.0;
}

double integerRound(const double value)
{
  return std::floor(value + 0.5);
}

bool ldFactorization(
  const Eigen::MatrixXd & covariance,
  Eigen::MatrixXd & lower,
  Eigen::VectorXd & diagonal)
{
  const int dimension = static_cast<int>(covariance.rows());
  Eigen::MatrixXd working = covariance;
  lower.setZero(dimension, dimension);
  diagonal.resize(dimension);
  for (int row = dimension - 1; row >= 0; --row) {
    diagonal(row) = working(row, row);
    if (!std::isfinite(diagonal(row)) || diagonal(row) <= 0.0) {
      return false;
    }
    const double root = std::sqrt(diagonal(row));
    for (int column = 0; column <= row; ++column) {
      lower(row, column) = working(row, column) / root;
    }
    for (int column = 0; column < row; ++column) {
      for (int inner = 0; inner <= column; ++inner) {
        working(column, inner) -= lower(row, inner) * lower(row, column);
      }
    }
    for (int column = 0; column <= row; ++column) {
      lower(row, column) /= lower(row, row);
    }
  }
  return lower.allFinite() && diagonal.allFinite();
}

void integerGauss(
  Eigen::MatrixXd & lower,
  Eigen::MatrixXd & transform,
  const int row,
  const int column)
{
  const int dimension = static_cast<int>(lower.rows());
  const int multiplier = static_cast<int>(integerRound(lower(row, column)));
  if (multiplier == 0) {
    return;
  }
  for (int index = row; index < dimension; ++index) {
    lower(index, column) -= static_cast<double>(multiplier) * lower(index, row);
  }
  for (int index = 0; index < dimension; ++index) {
    transform(index, column) -= static_cast<double>(multiplier) * transform(index, row);
  }
}

void permute(
  Eigen::MatrixXd & lower,
  Eigen::VectorXd & diagonal,
  const int column,
  const double delta,
  Eigen::MatrixXd & transform)
{
  const int dimension = static_cast<int>(lower.rows());
  const double eta = diagonal(column) / delta;
  const double lambda = diagonal(column + 1) * lower(column + 1, column) / delta;
  diagonal(column) = eta * diagonal(column + 1);
  diagonal(column + 1) = delta;
  for (int index = 0; index < column; ++index) {
    const double first = lower(column, index);
    const double second = lower(column + 1, index);
    lower(column, index) = -lower(column + 1, column) * first + second;
    lower(column + 1, index) = eta * first + lambda * second;
  }
  lower(column + 1, column) = lambda;
  for (int index = column + 2; index < dimension; ++index) {
    std::swap(lower(index, column), lower(index, column + 1));
  }
  for (int index = 0; index < dimension; ++index) {
    std::swap(transform(index, column), transform(index, column + 1));
  }
}

void reduce(
  Eigen::MatrixXd & lower,
  Eigen::VectorXd & diagonal,
  Eigen::MatrixXd & transform)
{
  const int dimension = static_cast<int>(lower.rows());
  int column = dimension - 2;
  int boundary = dimension - 2;
  while (column >= 0) {
    if (column <= boundary) {
      for (int row = column + 1; row < dimension; ++row) {
        integerGauss(lower, transform, row, column);
      }
    }
    const double delta = diagonal(column) +
      lower(column + 1, column) * lower(column + 1, column) * diagonal(column + 1);
    if (delta + 1.0e-6 < diagonal(column + 1)) {
      permute(lower, diagonal, column, delta, transform);
      boundary = column;
      column = dimension - 2;
    } else {
      --column;
    }
  }
}

bool search(
  const Eigen::MatrixXd & lower,
  const Eigen::VectorXd & diagonal,
  const Eigen::VectorXd & transformed_float,
  const int candidate_count,
  Eigen::MatrixXd & transformed_candidates,
  Eigen::VectorXd & squared_norms)
{
  const int dimension = static_cast<int>(lower.rows());
  Eigen::MatrixXd accumulated = Eigen::MatrixXd::Zero(dimension, dimension);
  Eigen::VectorXd distance(dimension);
  Eigen::VectorXd center(dimension);
  Eigen::VectorXd candidate(dimension);
  Eigen::VectorXd step(dimension);
  transformed_candidates.resize(dimension, candidate_count);
  squared_norms = Eigen::VectorXd::Constant(
    candidate_count, std::numeric_limits<double>::infinity());

  int level = dimension - 1;
  int found = 0;
  int maximum_index = 0;
  double maximum_distance = std::numeric_limits<double>::infinity();
  distance(level) = 0.0;
  center(level) = transformed_float(level);
  candidate(level) = integerRound(center(level));
  double offset = center(level) - candidate(level);
  step(level) = sign(offset);

  int loop = 0;
  for (; loop < kLoopMaximum; ++loop) {
    const double new_distance = distance(level) + offset * offset / diagonal(level);
    if (new_distance < maximum_distance) {
      if (level != 0) {
        --level;
        distance(level) = new_distance;
        for (int index = 0; index <= level; ++index) {
          accumulated(level, index) = accumulated(level + 1, index) +
            (candidate(level + 1) - center(level + 1)) * lower(level + 1, index);
        }
        center(level) = transformed_float(level) + accumulated(level, level);
        candidate(level) = integerRound(center(level));
        offset = center(level) - candidate(level);
        step(level) = sign(offset);
      } else {
        if (found < candidate_count) {
          if (found == 0 || new_distance > squared_norms(maximum_index)) {
            maximum_index = found;
          }
          transformed_candidates.col(found) = candidate;
          squared_norms(found) = new_distance;
          ++found;
        } else {
          if (new_distance < squared_norms(maximum_index)) {
            transformed_candidates.col(maximum_index) = candidate;
            squared_norms(maximum_index) = new_distance;
            squared_norms.maxCoeff(&maximum_index);
          }
          maximum_distance = squared_norms(maximum_index);
        }
        candidate(0) += step(0);
        offset = center(0) - candidate(0);
        step(0) = -step(0) - sign(step(0));
      }
    } else {
      if (level == dimension - 1) {
        break;
      }
      ++level;
      candidate(level) += step(level);
      offset = center(level) - candidate(level);
      step(level) = -step(level) - sign(step(level));
    }
  }
  if (loop >= kLoopMaximum || found < candidate_count) {
    return false;
  }
  for (int left = 0; left < candidate_count - 1; ++left) {
    for (int right = left + 1; right < candidate_count; ++right) {
      if (squared_norms(left) <= squared_norms(right)) {
        continue;
      }
      std::swap(squared_norms(left), squared_norms(right));
      transformed_candidates.col(left).swap(transformed_candidates.col(right));
    }
  }
  return transformed_candidates.allFinite() && squared_norms.allFinite();
}

}  // namespace

bool lambda(
  const Eigen::VectorXd & float_ambiguities,
  const Eigen::MatrixXd & covariance,
  const int candidate_count,
  LambdaResult & result)
{
  const int dimension = static_cast<int>(float_ambiguities.size());
  if (dimension <= 0 || candidate_count <= 0 || covariance.rows() != dimension ||
    covariance.cols() != dimension || !float_ambiguities.allFinite() ||
    !covariance.allFinite())
  {
    return false;
  }
  Eigen::MatrixXd lower;
  Eigen::VectorXd diagonal;
  if (!ldFactorization(covariance, lower, diagonal)) {
    return false;
  }
  Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(dimension, dimension);
  reduce(lower, diagonal, transform);
  const Eigen::VectorXd transformed_float = transform.transpose() * float_ambiguities;
  Eigen::MatrixXd transformed_candidates;
  Eigen::VectorXd squared_norms;
  if (!search(
      lower, diagonal, transformed_float, candidate_count,
      transformed_candidates, squared_norms))
  {
    return false;
  }
  const Eigen::FullPivLU<Eigen::MatrixXd> solver(transform.transpose());
  if (!solver.isInvertible()) {
    return false;
  }
  const Eigen::MatrixXd candidates = solver.solve(transformed_candidates);
  if (!candidates.allFinite()) {
    return false;
  }
  result.candidates = candidates;
  result.squared_norms = squared_norms;
  result.conditional_variances = diagonal;
  return true;
}

}  // namespace rtklib
}  // namespace fgo_gil_localizer
