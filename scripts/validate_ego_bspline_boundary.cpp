#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <iostream>

// Compile against the actual patched upstream uniform_bspline.cpp, not a
// second implementation of the planner. Boundary fitting is safety relevant.
int main() {
  using ego_planner::UniformBspline;
  const double dt = 0.4 / 0.85 * 1.2;
  for (double speed : {0.0, 0.12, 0.2}) {
    for (double accel : {0.0, 0.85, -1.2}) {
      std::vector<Eigen::Vector3d> samples;
      for (int i = 0; i < 12; ++i) samples.emplace_back(i * 0.3, 0.0, 0.0);
      std::vector<Eigen::Vector3d> derivatives = {
        {speed, 0, 0}, {0, 0, 0}, {accel, 0, 0}, {0, 0, 0}};
      Eigen::MatrixXd controls;
      UniformBspline::parameterizeToBspline(dt, samples, derivatives, controls);
      UniformBspline spline(controls, 3, dt);
      spline.lengthenTime(2.0);
      const auto velocity = spline.getDerivative();
      auto v = velocity;
      auto a = v.getDerivative();
      const double error = (spline.evaluateDeBoorT(0)-samples.front()).norm() +
        (v.evaluateDeBoorT(0)-derivatives[0]).norm() +
        (a.evaluateDeBoorT(0)-derivatives[2]).norm() +
        (spline.evaluateDeBoorT(spline.getTimeSum())-samples.back()).norm() +
        v.evaluateDeBoorT(spline.getTimeSum()).norm() +
        a.evaluateDeBoorT(spline.getTimeSum()).norm();
      std::cout << "v=" << speed << " a=" << accel << " boundary_error=" << error << '\n';
      if (!std::isfinite(error) || error > 1e-9) return 1;
    }
  }
  std::cout << "exact boundary regression: PASS\n";
}
