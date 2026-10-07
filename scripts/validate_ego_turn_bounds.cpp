#include <bspline_opt/vehicle_turn_bounds.h>
#include <iostream>
#include <cmath>
int main() {
  using ego_planner::UniformBspline;
  for (double yaw : {0.,.3,1.5707963267948966}) for (bool curve : {false,true}) {
    Eigen::MatrixXd q(3,20);q.setZero();
    for (int i=0;i<20;++i) {
      q(0,i)=curve?3*std::sin(i*.02):i*.02;
      q(1,i)=curve?3*(1-std::cos(i*.02)):0.;
    }
    Eigen::Matrix2d rotation;rotation << std::cos(yaw),-std::sin(yaw),std::sin(yaw),std::cos(yaw);
    q.topRows(2)=rotation*q.topRows(2);q.row(0).array()+=.125;q.row(1).array()+=.002;
    bool valid=ego_planner::certifyVehicleTurnBounds(UniformBspline(q,3,.2),1.,.7,1.4,1.8);
    std::cout << "broad_curve=" << curve << " certified=" << valid << '\n';
    if (!valid)return 1;
  }
  // Individual v/yaw/curvature limits can all pass while the original
  // corridor guard's coupled |v*w| cap fails. Test the continuous core gate.
  Eigen::MatrixXd circle(3,30);circle.setZero();
  for(int i=0;i<30;++i) {circle(0,i)=2*std::sin(i*.08);circle(1,i)=2*(1-std::cos(i*.08));}
  UniformBspline lateral(circle,3,.2);
  bool without_coupling=ego_planner::certifyVehicleTurnBounds(lateral,1.,.7,1.4,1.8);
  bool with_coupling=ego_planner::certifyVehicleTurnBounds(lateral,1.,.7,1.4,1.8,.25);
  std::cout << "individual_turn_bounds=" << without_coupling << " coupled_lateral_bound=" << with_coupling << '\n';
  if(!without_coupling || with_coupling)return 1;
  // Cubic velocity changes sign twice inside a 0.1s span. The old 0.05s
  // grid skips the tiny midspan speed and sees legal endpoint yaw rates.
  const double dt=.1;
  Eigen::Vector3d p=Eigen::Vector3d::Zero(),v(2e-5,.00315,0),a(0,-.15,0),j(0,3,0);
  Eigen::MatrixXd q(3,4);
  q.col(1)=p-dt*dt*a/6.;q.col(0)=q.col(1)-dt*v+.5*dt*dt*a;
  q.col(2)=q.col(1)+dt*v+.5*dt*dt*a;q.col(3)=j*dt*dt*dt+3*q.col(2)-3*q.col(1)+q.col(0);
  UniformBspline spline(q,3,dt);auto velocity=spline.getDerivative(),acceleration=velocity.getDerivative();
  double sampled=0,dense=0;
  for(double t=0;t<=dt+1e-8;t+=.0001) {
    auto vv=velocity.evaluateDeBoorT(std::min(t,dt)),aa=acceleration.evaluateDeBoorT(std::min(t,dt));
    double speed=vv.head<2>().norm();if(speed<1e-3)continue;
    double w=std::abs((vv.x()*aa.y()-vv.y()*aa.x())/(speed*speed));
    dense=std::max(dense,w);
    if(std::abs(t)<1e-8 || std::abs(t-.05)<1e-8 || std::abs(t-.1)<1e-8)sampled=std::max(sampled,w);
  }
  bool certified=ego_planner::certifyVehicleTurnBounds(spline,100.,.7,1.4,1.8);
  std::cout << "old_sampled_max_yaw_rate=" << sampled << " dense_max=" << dense << " certified=" << certified << '\n';
  return sampled<.7 && dense>.7 && !certified?0:1;
}
