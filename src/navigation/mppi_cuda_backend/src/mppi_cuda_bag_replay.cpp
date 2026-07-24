#include "mppi_cuda_backend/mppi_cuda_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

namespace
{

constexpr char kLocalCostmapTopic[] = "/local_costmap/costmap";
constexpr char kPathTopic[] = "/gps_waypoint_dispatcher/path_map";
constexpr char kCommandTopic[] = "/cmd_vel_nav";
constexpr char kTfTopic[] = "/tf";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kAuthorityName[] = "mppi_cuda_bag_replay";

struct Options
{
  std::string bag_path;
  std::string output_path;
  std::size_t max_frames{0U};
  std::size_t batch_size{4096U};
  std::size_t time_steps{48U};
  float prune_distance_m{4.0F};
  std::size_t lookahead_points{6U};
  bool track_unknown{false};
};

struct Samples
{
  void add(float value) { values.push_back(value); }

  float mean() const
  {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    float total = 0.0F;
    for (const float value : values) {
      total += value;
    }
    return total / static_cast<float>(values.size());
  }

  float percentile(float quantile) const
  {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    auto sorted = values;
    const auto index = static_cast<std::size_t>(quantile * static_cast<float>(sorted.size() - 1U));
    std::nth_element(sorted.begin(), sorted.begin() + index, sorted.end());
    return sorted[index];
  }

  std::vector<float> values;
};

struct Counters
{
  std::size_t costmaps_seen{0U};
  std::size_t evaluated{0U};
  std::size_t no_path{0U};
  std::size_t no_command{0U};
  std::size_t stale_command{0U};
  std::size_t tf_failure{0U};
  std::size_t empty_local_path{0U};
  std::size_t all_collide{0U};
};

struct PathState
{
  nav_msgs::msg::Path path;
  std::size_t prune_index{0U};
};

struct LocalPathInput
{
  float robot_x{0.0F};
  float robot_y{0.0F};
  float robot_yaw{0.0F};
  float target_x{0.0F};
  float target_y{0.0F};
  float goal_x{0.0F};
  float goal_y{0.0F};
};

void printUsage(const char * program)
{
  std::cout << "Usage: " << program << " <bag_path> [options]\\n"
            << "  --output <csv>             Write one row for every evaluated costmap frame\\n"
            << "  --max-frames <count>       Stop after this many evaluated frames (0 = all)\\n"
            << "  --batch-size <count>       CUDA sample count (default: 4096)\\n"
            << "  --time-steps <count>       CUDA horizon steps (default: 48)\\n"
            << "  --prune-distance <metres>  Local path horizon (default: 4.0)\\n"
            << "  --lookahead-points <count> CUDA target index in the local path (default: 6)\\n"
            << "  --track-unknown            Treat OccupancyGrid unknown cells as collisions\\n";
}

std::size_t parseSize(const std::string & value, const char * flag)
{
  try {
    const auto parsed = std::stoull(value);
    if (parsed == 0U) {
      throw std::invalid_argument("must be greater than zero");
    }
    return static_cast<std::size_t>(parsed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(flag) + " requires a positive integer");
  }
}

float parsePositiveFloat(const std::string & value, const char * flag)
{
  try {
    const float parsed = std::stof(value);
    if (!std::isfinite(parsed) || parsed <= 0.0F) {
      throw std::invalid_argument("must be finite and positive");
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(flag) + " requires a positive number");
  }
}

Options parseOptions(int argc, char ** argv)
{
  if (argc < 2) {
    throw std::invalid_argument("bag path is required");
  }
  Options options;
  options.bag_path = argv[1];
  for (int index = 2; index < argc; ++index) {
    const std::string flag = argv[index];
    const auto next = [&]() {
        if (++index >= argc) {
          throw std::invalid_argument(flag + " requires a value");
        }
        return std::string(argv[index]);
      };
    if (flag == "--output") {
      options.output_path = next();
    } else if (flag == "--max-frames") {
      const auto value = next();
      try {
        options.max_frames = static_cast<std::size_t>(std::stoull(value));
      } catch (const std::exception &) {
        throw std::invalid_argument("--max-frames requires a non-negative integer");
      }
    } else if (flag == "--batch-size") {
      options.batch_size = parseSize(next(), "--batch-size");
    } else if (flag == "--time-steps") {
      options.time_steps = parseSize(next(), "--time-steps");
    } else if (flag == "--prune-distance") {
      options.prune_distance_m = parsePositiveFloat(next(), "--prune-distance");
    } else if (flag == "--lookahead-points") {
      options.lookahead_points = parseSize(next(), "--lookahead-points");
    } else if (flag == "--track-unknown") {
      options.track_unknown = true;
    } else if (flag == "--help" || flag == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("unknown option: " + flag);
    }
  }
  return options;
}

template<typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serializer;
  MessageT message;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

rclcpp::Time headerStamp(const std_msgs::msg::Header & header)
{
  return rclcpp::Time(header.stamp, RCL_ROS_TIME);
}

rclcpp::Time messageStamp(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
{
  return rclcpp::Time(bag_message->time_stamp, RCL_ROS_TIME);
}

float distance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(
    static_cast<float>(first.pose.position.x - second.pose.position.x),
    static_cast<float>(first.pose.position.y - second.pose.position.y));
}

std::size_t integratedDistanceEnd(
  const nav_msgs::msg::Path & path, std::size_t begin, float maximum_distance)
{
  float integrated_distance = 0.0F;
  std::size_t end = begin + 1U;
  while (end < path.poses.size()) {
    integrated_distance += distance(path.poses[end - 1U], path.poses[end]);
    if (integrated_distance > maximum_distance) {
      break;
    }
    ++end;
  }
  return end;
}

bool isInsideCostmap(const nav_msgs::msg::OccupancyGrid & costmap, float x, float y)
{
  const float local_x = x - static_cast<float>(costmap.info.origin.position.x);
  const float local_y = y - static_cast<float>(costmap.info.origin.position.y);
  return local_x >= 0.0F && local_y >= 0.0F &&
         local_x < static_cast<float>(costmap.info.width) * costmap.info.resolution &&
         local_y < static_cast<float>(costmap.info.height) * costmap.info.resolution;
}

std::vector<unsigned char> makeRawCostmap(const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  std::vector<unsigned char> raw;
  raw.reserve(occupancy_grid.data.size());
  for (const std::int8_t occupancy : occupancy_grid.data) {
    if (occupancy < 0) {
      raw.push_back(255U);
      continue;
    }
    // Nav2's OccupancyGrid publisher compresses Costmap2D's 0..254 costs to
    // 0..100. Restore the lethal/inscribed threshold used by the CUDA backend.
    const float scaled = static_cast<float>(occupancy) * 254.0F / 100.0F;
    raw.push_back(static_cast<unsigned char>(std::lround(std::min(scaled, 254.0F))));
  }
  return raw;
}

std::optional<LocalPathInput> makeLocalPathInput(
  const nav_msgs::msg::OccupancyGrid & costmap, PathState & path_state,
  tf2_ros::Buffer & tf_buffer, const Options & options)
{
  if (path_state.path.poses.empty() || path_state.path.header.frame_id.empty() ||
    costmap.header.frame_id.empty())
  {
    return std::nullopt;
  }
  const auto stamp = headerStamp(costmap.header);
  try {
    const auto local_from_base = tf_buffer.lookupTransform(
      costmap.header.frame_id, "base_link", stamp, tf2::durationFromSec(0.0));
    geometry_msgs::msg::PoseStamped robot_local;
    robot_local.header.frame_id = costmap.header.frame_id;
    robot_local.header.stamp = costmap.header.stamp;
    robot_local.pose.position.x = local_from_base.transform.translation.x;
    robot_local.pose.position.y = local_from_base.transform.translation.y;
    robot_local.pose.position.z = local_from_base.transform.translation.z;
    robot_local.pose.orientation = local_from_base.transform.rotation;

    const auto path_from_local = tf_buffer.lookupTransform(
      path_state.path.header.frame_id, costmap.header.frame_id, stamp, tf2::durationFromSec(0.0));
    geometry_msgs::msg::PoseStamped robot_path;
    tf2::doTransform(robot_local, robot_path, path_from_local);

    const std::size_t path_size = path_state.path.poses.size();
    path_state.prune_index = std::min(path_state.prune_index, path_size - 1U);
    const float max_search_distance = std::max(
      static_cast<float>(costmap.info.width) * costmap.info.resolution,
      static_cast<float>(costmap.info.height) * costmap.info.resolution) * 0.5F;
    const std::size_t search_end = integratedDistanceEnd(
      path_state.path, path_state.prune_index, max_search_distance);
    auto closest = path_state.prune_index;
    float closest_distance = std::numeric_limits<float>::max();
    for (std::size_t index = path_state.prune_index; index < search_end; ++index) {
      const float candidate_distance = distance(robot_path, path_state.path.poses[index]);
      if (candidate_distance < closest_distance) {
        closest = index;
        closest_distance = candidate_distance;
      }
    }
    path_state.prune_index = closest;

    const auto local_from_path = tf_buffer.lookupTransform(
      costmap.header.frame_id, path_state.path.header.frame_id, stamp, tf2::durationFromSec(0.0));
    const std::size_t local_end = integratedDistanceEnd(
      path_state.path, closest, options.prune_distance_m);
    std::vector<geometry_msgs::msg::PoseStamped> local_poses;
    local_poses.reserve(local_end - closest);
    for (std::size_t index = closest; index < local_end; ++index) {
      auto pose = path_state.path.poses[index];
      pose.header.frame_id = path_state.path.header.frame_id;
      pose.header.stamp = costmap.header.stamp;
      geometry_msgs::msg::PoseStamped transformed;
      tf2::doTransform(pose, transformed, local_from_path);
      if (!isInsideCostmap(
          costmap, static_cast<float>(transformed.pose.position.x),
          static_cast<float>(transformed.pose.position.y)))
      {
        break;
      }
      local_poses.push_back(std::move(transformed));
    }
    if (local_poses.empty()) {
      return std::nullopt;
    }

    const auto & target = local_poses.at(std::min(options.lookahead_points, local_poses.size() - 1U));
    const auto & goal = local_poses.back();
    return LocalPathInput{
      static_cast<float>(robot_local.pose.position.x),
      static_cast<float>(robot_local.pose.position.y),
      static_cast<float>(tf2::getYaw(robot_local.pose.orientation)),
      static_cast<float>(target.pose.position.x),
      static_cast<float>(target.pose.position.y),
      static_cast<float>(goal.pose.position.x),
      static_cast<float>(goal.pose.position.y),
    };
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

void writeSummary(
  const Counters & counters, const Samples & gpu_ms, const Samples & vx_error,
  const Samples & wz_error, const Samples & command_age)
{
  std::cout << std::fixed << std::setprecision(3)
            << "costmaps_seen=" << counters.costmaps_seen
            << " evaluated=" << counters.evaluated
            << " all_collide=" << counters.all_collide
            << " no_path=" << counters.no_path
            << " no_command=" << counters.no_command
            << " stale_command=" << counters.stale_command
            << " tf_or_path_failure=" << (counters.tf_failure + counters.empty_local_path)
            << '\n';
  if (counters.evaluated == 0U) {
    return;
  }
  std::cout << "gpu_ms_mean=" << gpu_ms.mean()
            << " gpu_ms_p50=" << gpu_ms.percentile(0.50F)
            << " gpu_ms_p95=" << gpu_ms.percentile(0.95F)
            << " abs_vx_error_mean=" << vx_error.mean()
            << " abs_vx_error_p95=" << vx_error.percentile(0.95F)
            << " abs_wz_error_mean=" << wz_error.mean()
            << " abs_wz_error_p95=" << wz_error.percentile(0.95F)
            << " command_age_ms_p95=" << command_age.percentile(0.95F) * 1000.0F
            << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    if (!mppi_cuda_backend::CudaMppiBackend::isAvailable()) {
      std::cerr << "No CUDA device available" << std::endl;
      return EXIT_FAILURE;
    }
    rclcpp::init(argc, argv);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer tf_buffer(clock, tf2::durationFromSec(30.0));
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = options.bag_path;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);

    std::ofstream output;
    if (!options.output_path.empty()) {
      output.open(options.output_path);
      if (!output) {
        throw std::runtime_error("unable to open output CSV: " + options.output_path);
      }
      output << "stamp_ns,gpu_ms,cpu_vx,cpu_wz,gpu_vx,gpu_wz,abs_vx_error,abs_wz_error,"
             << "command_age_ms,all_trajectories_collide\\n";
    }

    mppi_cuda_backend::SamplingConfig config;
    config.batch_size = options.batch_size;
    config.time_steps = options.time_steps;
    config.vx_max = 1.5F;
    config.wz_max = 0.7F;
    config.vx_std = 0.28F;
    config.wz_std = 0.22F;
    config.temperature = 0.45F;
    config.gamma = 0.015F;
    config.path_weight = 16.0F;
    config.goal_weight = 5.0F;
    mppi_cuda_backend::CudaMppiBackend backend;
    PathState path_state;
    std::optional<geometry_msgs::msg::Twist> latest_command;
    std::optional<rclcpp::Time> latest_command_stamp;
    Counters counters;
    Samples gpu_ms;
    Samples vx_error;
    Samples wz_error;
    Samples command_age;

    while (reader.has_next()) {
      const auto bag_message = reader.read_next();
      if (bag_message->topic_name == kTfTopic || bag_message->topic_name == kTfStaticTopic) {
        const auto message = deserialize<tf2_msgs::msg::TFMessage>(bag_message);
        for (const auto & transform : message.transforms) {
          if (bag_message->topic_name == kTfStaticTopic) {
            tf_buffer.setTransform(transform, kAuthorityName, true);
          } else {
            tf_buffer.setTransform(transform, kAuthorityName);
          }
        }
        continue;
      }
      if (bag_message->topic_name == kPathTopic) {
        path_state.path = deserialize<nav_msgs::msg::Path>(bag_message);
        path_state.prune_index = 0U;
        continue;
      }
      if (bag_message->topic_name == kCommandTopic) {
        latest_command = deserialize<geometry_msgs::msg::Twist>(bag_message);
        latest_command_stamp = messageStamp(bag_message);
        continue;
      }
      if (bag_message->topic_name != kLocalCostmapTopic) {
        continue;
      }

      ++counters.costmaps_seen;
      if (path_state.path.poses.empty()) {
        ++counters.no_path;
        continue;
      }
      if (!latest_command || !latest_command_stamp) {
        ++counters.no_command;
        continue;
      }
      const auto costmap = deserialize<nav_msgs::msg::OccupancyGrid>(bag_message);
      const auto stamp = headerStamp(costmap.header);
      const float age_s = static_cast<float>((stamp - *latest_command_stamp).seconds());
      if (age_s < -0.001F || age_s > 0.25F) {
        ++counters.stale_command;
        continue;
      }
      const auto local_input = makeLocalPathInput(costmap, path_state, tf_buffer, options);
      if (!local_input) {
        ++counters.empty_local_path;
        continue;
      }
      const auto raw_costmap = makeRawCostmap(costmap);
      if (raw_costmap.size() != static_cast<std::size_t>(costmap.info.width) * costmap.info.height) {
        throw std::runtime_error("local costmap data size does not match metadata");
      }

      mppi_cuda_backend::OptimizerInput input;
      input.robot_x = local_input->robot_x;
      input.robot_y = local_input->robot_y;
      input.robot_yaw = local_input->robot_yaw;
      input.path_target_x = local_input->target_x;
      input.path_target_y = local_input->target_y;
      input.goal_x = local_input->goal_x;
      input.goal_y = local_input->goal_y;
      input.nominal_vx.assign(config.time_steps, static_cast<float>(latest_command->linear.x));
      input.nominal_wz.assign(config.time_steps, static_cast<float>(latest_command->angular.z));
      input.costmap = {
        raw_costmap.data(),
        costmap.info.width,
        costmap.info.height,
        costmap.info.resolution,
        static_cast<float>(costmap.info.origin.position.x),
        static_cast<float>(costmap.info.origin.position.y),
        options.track_unknown,
      };
      const auto result = backend.optimize(config, input);
      const float vx_difference = std::fabs(result.control_vx.front() - input.nominal_vx.front());
      const float wz_difference = std::fabs(result.control_wz.front() - input.nominal_wz.front());
      ++counters.evaluated;
      counters.all_collide += result.all_trajectories_collide ? 1U : 0U;
      gpu_ms.add(result.gpu_elapsed_ms);
      vx_error.add(vx_difference);
      wz_error.add(wz_difference);
      command_age.add(age_s);
      if (output) {
        output << std::fixed << std::setprecision(6)
               << stamp.nanoseconds() << ',' << result.gpu_elapsed_ms << ','
               << input.nominal_vx.front() << ',' << input.nominal_wz.front() << ','
               << result.control_vx.front() << ',' << result.control_wz.front() << ','
               << vx_difference << ',' << wz_difference << ',' << age_s * 1000.0F << ','
               << (result.all_trajectories_collide ? 1 : 0) << '\n';
      }
      if (options.max_frames != 0U && counters.evaluated >= options.max_frames) {
        break;
      }
    }
    writeSummary(counters, gpu_ms, vx_error, wz_error, command_age);
    rclcpp::shutdown();
    return counters.evaluated == 0U ? EXIT_FAILURE : EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "mppi_cuda_bag_replay failed: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return EXIT_FAILURE;
  }
}
