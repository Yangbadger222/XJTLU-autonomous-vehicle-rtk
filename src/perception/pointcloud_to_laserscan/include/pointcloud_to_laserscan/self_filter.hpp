#ifndef POINTCLOUD_TO_LASERSCAN__SELF_FILTER_HPP_
#define POINTCLOUD_TO_LASERSCAN__SELF_FILTER_HPP_

namespace pointcloud_to_laserscan
{

struct SelfFilterBox
{
  bool enabled{false};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};

  bool valid() const
  {
    return !enabled || (min_x < max_x && min_y < max_y);
  }

  bool contains(double x, double y) const
  {
    return enabled && x >= min_x && x <= max_x && y >= min_y && y <= max_y;
  }
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__SELF_FILTER_HPP_
