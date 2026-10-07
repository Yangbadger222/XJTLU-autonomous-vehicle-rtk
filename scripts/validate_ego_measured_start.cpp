#include <plan_manage/planner_interface.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cstdlib>

int main() {
  int successes=0, count=0;
  for (double lateral : {0., .0005, -.0005, .01}) {
    ego_planner::PlannerInterface planner;
    planner.initVehicleParam(.85,.85,3.,.7,1.);
    planner.setPlanningAccelerationFraction(.5);
    planner.setPlanningSpeedFraction(.8);
    planner.initEsdfMap(30,30,1,.3,Eigen::Vector3d(-15,-15,0),std::hypot(.33,.305));
    planner.setInputGridGeometry(100,100,Eigen::Vector2d(-15,-15));
    ego_planner::PathPoint state;state.x=.002;state.y=lateral;state.z=-.00003;
    planner.setCurrentVehicleState(state,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    std::vector<ego_planner::PathPoint> reference;
    for(int i=0;i<17;++i) { ego_planner::PathPoint p;p.x=state.x+(1.2-state.x)*i/16.;p.y=i==0?lateral:0.;reference.push_back(p); }
    planner.setPathPoint(reference);
    std::vector<ego_planner::ObstacleInfo> obstacles;
    if(const char* file=std::getenv("RESEARCH_GROUND_FIXTURE")) {
      std::ifstream input(file);int cell;
      for(int row=0;row<100;++row)for(int col=0;col<100;++col) {
        if(!(input>>cell)) return 2;
        if(cell!=0) obstacles.push_back({-15+(col+.5)*.3,-15+(row+.5)*.3,0.});
      }
    }
    planner.setObstacles(obstacles);
    planner.makePlan();
    std::vector<ego_planner::PathPoint> planned;planner.getLocalPlanTrajResults(planned);
    std::cout << "measured lateral=" << lateral << " sample_count=" << planned.size() << '\n';
    ++count;if(planned.size()>10)++successes;
  }
  {
    ego_planner::PlannerInterface planner;planner.initVehicleParam(.85,.85,3.,.7,1.);
    planner.setPlanningAccelerationFraction(.5);planner.setPlanningSpeedFraction(.8);
    planner.initEsdfMap(30,30,1,.3,Eigen::Vector3d(-15,-15,0),std::hypot(.33,.305));
    planner.setInputGridGeometry(100,100,Eigen::Vector2d(-15,-15));
    ego_planner::PathPoint state;state.x=1.2;state.y=state.z=0.;
    planner.setCurrentVehicleState(state,Eigen::Vector3d(.12,0,0),Eigen::Vector3d::Zero());
    std::vector<ego_planner::PathPoint> reference;
    for(int i=0;i<12;++i){ego_planner::PathPoint p;p.x=i*.3;p.y=0.;reference.push_back(p);}
    planner.setPathPoint(reference);std::vector<ego_planner::ObstacleInfo> obstacles;planner.setObstacles(obstacles);
    planner.makePlan();std::vector<ego_planner::PathPoint> planned;planner.getLocalPlanTrajResults(planned);
    const bool good=!planned.empty() && std::abs(planned.front().x-state.x)<1e-9 && std::abs(planned.front().vx-.12)<1e-9;
    std::cout<<"measured_pose_origin_preserved="<<good<<'\n';++count;if(good)++successes;
  }
  for(double step : {.1,(M_PI/3)/11}) {
    ego_planner::PlannerInterface planner;planner.initVehicleParam(.85,.85,3.,.7,1.);
    planner.setPlanningAccelerationFraction(.5);planner.setPlanningSpeedFraction(.8);
    planner.initEsdfMap(30,30,1,.3,Eigen::Vector3d(-15,-15,0),std::hypot(.33,.305));
    planner.setInputGridGeometry(100,100,Eigen::Vector2d(-15,-15));
    ego_planner::PathPoint state;state.x=state.y=state.z=0.;
    planner.setCurrentVehicleState(state,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    std::vector<ego_planner::PathPoint> reference;
    for(int i=0;i<12;++i){ego_planner::PathPoint p;p.x=3*std::sin(i*step);p.y=3*(1-std::cos(i*step));reference.push_back(p);}
    planner.setPathPoint(reference);std::vector<ego_planner::ObstacleInfo> obstacles;planner.setObstacles(obstacles);
    planner.makePlan();std::vector<ego_planner::PathPoint> planned;planner.getLocalPlanTrajResults(planned);
    std::cout<<"arc_step="<<step<<" sample_count="<<planned.size()<<'\n';++count;if(planned.size()>10)++successes;
  }
  std::cout << "measured_start_success=" << successes << '/' << count << '\n';
  return successes==count?0:1;
}
