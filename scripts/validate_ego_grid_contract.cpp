#include <plan_env/grid_map.h>
#include <iostream>
#include <cmath>
int main() {
  const double resolution=static_cast<float>(.3);
  GridMap2D grid(resolution,Eigen::Vector2i(30,30));
  grid.setInputGeometry(100,100,Eigen::Vector2d(-15,-15));grid.setInflateRadius(std::hypot(.33,.305));
  for(int row=0;row<100;++row)for(int col=0;col<100;++col) {
    const Eigen::Vector2i index(col,row);
    if(grid.worldToGrid(grid.gridToWorld(index))!=index)return 1;
  }
  if(grid.worldToGrid(Eigen::Vector2d(-15-1e-9,0)).x()!=-1)return 1;
  grid.setObstacle(Eigen::Vector2i(54,50));
  if(grid.getInflateOccupancy(Eigen::Vector2d(.6,.15)))return 1;
  if(!grid.getInflateOccupancy(Eigen::Vector2d(.9,.15)))return 1;
  if(!grid.getInflateOccupancy(Eigen::Vector2d(-14.9,0)))return 1;
  std::cout << "10000 cell-center roundtrips, negative boundary, continuous obstacle-cell inflation: PASS\n";
}
