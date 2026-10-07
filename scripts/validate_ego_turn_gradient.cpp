#include <bspline_opt/bspline_optimizer.h>
#include <iostream>
#include <cmath>
namespace ego_planner {
struct VehicleCostGradientProbe {
  static void evaluate(BsplineOptimizer& optimizer, const Eigen::MatrixXd& q,
                       int curvature, double& cost, Eigen::MatrixXd& gradient) {
    if(curvature==2) optimizer.calLateralAccelCost(q,cost,gradient);
    else if (curvature==1) optimizer.calKappaCost(q,cost,gradient);
    else optimizer.calTurnCost(q,cost,gradient);
  }
};
}
int main() {
  ego_planner::BsplineOptimizer optimizer;
  optimizer.setParam(.85,.85);
  optimizer.setBsplineInterval(.1);
  optimizer.setVehicleTurnLimits(1.,.7,.25);
  double worst=0;
  for (double scale : {.0007,.04,.08}) for (double sign : {-1.,1.}) {
    Eigen::MatrixXd q(2,5);
    q << 0,scale,2*scale,3*scale,4*scale,
         0,0,.2*sign,.5*sign,.9*sign;
    for (int curvature : {0,1,2}) {
      double cost; Eigen::MatrixXd analytic;
      ego_planner::VehicleCostGradientProbe::evaluate(optimizer,q,curvature,cost,analytic);
      double error=0;
      for (int row=0;row<2;++row) for (int col=0;col<q.cols();++col) {
        const double epsilon=1e-8;
        auto plus=q,minus=q; plus(row,col)+=epsilon; minus(row,col)-=epsilon;
        double cplus,cminus; Eigen::MatrixXd ignored;
        ego_planner::VehicleCostGradientProbe::evaluate(optimizer,plus,curvature,cplus,ignored);
        ego_planner::VehicleCostGradientProbe::evaluate(optimizer,minus,curvature,cminus,ignored);
        double numeric=(cplus-cminus)/(2*epsilon);
        error=std::max(error,std::abs(numeric-analytic(row,col))/(1+std::abs(numeric)));
      }
      worst=std::max(worst,error);
      std::cout << "scale=" << scale << " sign=" << sign << " curvature=" << curvature << " cost=" << cost << " gradient_error=" << error << '\n';
    }
  }
  std::cout << "worst_relative_error=" << worst << '\n';
  return std::isfinite(worst) && worst < 1e-4 ? 0 : 1;
}
