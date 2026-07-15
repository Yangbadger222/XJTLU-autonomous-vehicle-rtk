#include "fgo_gil_localizer/gnss_time.hpp"
#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/time_sync.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "gnss_raw_msgs/msg/observation_epoch.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

namespace fgo_gil_localizer
{
namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

template<typename T>
diagnostic_msgs::msg::KeyValue numericKeyValue(const std::string & key, const T value)
{
  return keyValue(key, std::to_string(value));
}

TimeSyncState weakerState(const TimeSyncState first, const TimeSyncState second)
{
  return static_cast<TimeSyncState>(std::min(
           static_cast<std::uint8_t>(first), static_cast<std::uint8_t>(second)));
}

}  // namespace

class TimeSyncNode : public rclcpp::Node
{
public:
  TimeSyncNode()
  : Node("fgo_gil_time_sync")
  {
    declareParameters();
    readParameters();
    createInterfaces();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("topics.imu", "/livox/imu");
    declare_parameter<std::string>("topics.lidar", "/livox/lidar");
    declare_parameter<std::string>("topics.gnss_epoch", "/gnss/raw/observation_epoch");
    declare_parameter<std::string>("topics.gnss_pps", "/gnss/pps/time_reference");
    declare_parameter<std::string>("topics.imu_time_reference", "/livox/imu_time_reference");
    declare_parameter<std::string>("topics.lidar_time_reference", "/livox/lidar_time_reference");
    declare_parameter<std::string>(
      "topics.diagnostics", "/fgo_gil/time_sync_diagnostics");
    declare_parameter<std::string>("topics.status", "/fgo_gil/timing_status");
    declare_parameter<std::string>("imu_stamp_domain", "ros");
    declare_parameter<std::string>("lidar_stamp_domain", "ros");
    declare_parameter<int>("time_sync.window_size", 32);
    declare_parameter<int>("time_sync.min_coarse_samples", 5);
    declare_parameter<int>("time_sync.min_pps_samples", 2);
    declare_parameter<double>("time_sync.max_offset_jump_s", 0.5);
    declare_parameter<double>("time_sync.coarse_timeout_s", 5.0);
    declare_parameter<double>("time_sync.pps_timeout_s", 2.0);
    declare_parameter<double>("time_sync.coarse_uncertainty_floor_s", 0.02);
    declare_parameter<double>("time_sync.pps_uncertainty_floor_s", 1.0e-4);
    declare_parameter<double>("time_sync.gnss_reception_uncertainty_s", 0.02);
    declare_parameter<double>("time_sync.device_reference_uncertainty_s", 1.0e-3);
    declare_parameter<bool>("time_sync.device_reference_locked", true);
    declare_parameter<double>("time_sync.gnss_max_forward_gap_s", 10.0);
    declare_parameter<int>("imu_buffer.capacity", 4096);
    declare_parameter<double>("imu_buffer.max_gap_s", 0.05);
    declare_parameter<double>("lidar_max_gap_s", 0.5);
    declare_parameter<double>("diagnostics_period_s", 1.0);
  }

  void readParameters()
  {
    imu_topic_ = get_parameter("topics.imu").as_string();
    lidar_topic_ = get_parameter("topics.lidar").as_string();
    gnss_epoch_topic_ = get_parameter("topics.gnss_epoch").as_string();
    gnss_pps_topic_ = get_parameter("topics.gnss_pps").as_string();
    imu_time_reference_topic_ = get_parameter("topics.imu_time_reference").as_string();
    lidar_time_reference_topic_ = get_parameter("topics.lidar_time_reference").as_string();
    diagnostics_topic_ = get_parameter("topics.diagnostics").as_string();
    status_topic_ = get_parameter("topics.status").as_string();
    imu_stamp_domain_ = get_parameter("imu_stamp_domain").as_string();
    lidar_stamp_domain_ = get_parameter("lidar_stamp_domain").as_string();

    TimeSyncConfig time_config;
    time_config.window_size = positiveSizeParameter("time_sync.window_size");
    time_config.min_coarse_samples = positiveSizeParameter("time_sync.min_coarse_samples");
    time_config.min_pps_samples = positiveSizeParameter("time_sync.min_pps_samples");
    time_config.max_offset_jump_s = get_parameter("time_sync.max_offset_jump_s").as_double();
    time_config.coarse_timeout_s = get_parameter("time_sync.coarse_timeout_s").as_double();
    time_config.pps_timeout_s = get_parameter("time_sync.pps_timeout_s").as_double();
    time_config.coarse_uncertainty_floor_s =
      get_parameter("time_sync.coarse_uncertainty_floor_s").as_double();
    time_config.pps_uncertainty_floor_s =
      get_parameter("time_sync.pps_uncertainty_floor_s").as_double();
    pps_uncertainty_s_ = time_config.pps_uncertainty_floor_s;
    ros_to_gnss_ = std::make_unique<TimeSyncEstimator>(time_config);
    imu_to_ros_ = std::make_unique<TimeSyncEstimator>(time_config);
    lidar_to_ros_ = std::make_unique<TimeSyncEstimator>(time_config);

    gnss_reception_uncertainty_s_ =
      get_parameter("time_sync.gnss_reception_uncertainty_s").as_double();
    device_reference_uncertainty_s_ =
      get_parameter("time_sync.device_reference_uncertainty_s").as_double();
    device_reference_locked_ = get_parameter("time_sync.device_reference_locked").as_bool();
    gnss_time_tracker_ = std::make_unique<GnssTimeTracker>(
      get_parameter("time_sync.gnss_max_forward_gap_s").as_double());
    imu_buffer_ = std::make_unique<ImuSegmentBuffer>(
      ImuBufferConfig{
        positiveSizeParameter("imu_buffer.capacity"),
        get_parameter("imu_buffer.max_gap_s").as_double()});
    lidar_max_gap_s_ = get_parameter("lidar_max_gap_s").as_double();
    diagnostics_period_s_ = get_parameter("diagnostics_period_s").as_double();

    if ((imu_stamp_domain_ != "ros" && imu_stamp_domain_ != "device") ||
      (lidar_stamp_domain_ != "ros" && lidar_stamp_domain_ != "device") ||
      !std::isfinite(gnss_reception_uncertainty_s_) || gnss_reception_uncertainty_s_ < 0.0 ||
      !std::isfinite(device_reference_uncertainty_s_) || device_reference_uncertainty_s_ < 0.0 ||
      !std::isfinite(lidar_max_gap_s_) || lidar_max_gap_s_ <= 0.0 ||
      !std::isfinite(diagnostics_period_s_) || diagnostics_period_s_ <= 0.0)
    {
      throw std::invalid_argument("FGO-GIL time-sync parameters are outside valid bounds");
    }
  }

  std::size_t positiveSizeParameter(const std::string & name) const
  {
    const std::int64_t value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void createInterfaces()
  {
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    gnss_epoch_sub_ = create_subscription<gnss_raw_msgs::msg::ObservationEpoch>(
      gnss_epoch_topic_, 50,
      std::bind(&TimeSyncNode::onGnssEpoch, this, std::placeholders::_1));
    pps_sub_ = create_subscription<sensor_msgs::msg::TimeReference>(
      gnss_pps_topic_, 20,
      std::bind(&TimeSyncNode::onGnssPps, this, std::placeholders::_1));
    imu_time_reference_sub_ = create_subscription<sensor_msgs::msg::TimeReference>(
      imu_time_reference_topic_, 20,
      [this](sensor_msgs::msg::TimeReference::SharedPtr message) {
        observeDeviceReference(*message, *imu_to_ros_);
      });
    lidar_time_reference_sub_ = create_subscription<sensor_msgs::msg::TimeReference>(
      lidar_time_reference_topic_, 20,
      [this](sensor_msgs::msg::TimeReference::SharedPtr message) {
        observeDeviceReference(*message, *lidar_to_ros_);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TimeSyncNode::onImu, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TimeSyncNode::onLidar, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_s_),
      std::bind(&TimeSyncNode::publishDiagnostics, this));
  }

  void onGnssEpoch(const gnss_raw_msgs::msg::ObservationEpoch::SharedPtr message)
  {
    if (!isGnssClockReferenceReceiver(message->receiver)) {
      return;
    }
    const GnssTimeResult result =
      gnss_time_tracker_->accept(message->week, message->milliseconds_of_week);
    if (result == GnssTimeResult::Invalid || result == GnssTimeResult::Duplicate) {
      return;
    }
    if (result == GnssTimeResult::ResetAccepted) {
      ros_to_gnss_->reset();
    }
    const double ros_reception_s = stampSeconds(message->header.stamp);
    ros_to_gnss_->observe(
      {
        ros_reception_s,
        gnss_time_tracker_->latest()->absolute_seconds,
        gnss_reception_uncertainty_s_,
        false});
  }

  void onGnssPps(const sensor_msgs::msg::TimeReference::SharedPtr message)
  {
    ros_to_gnss_->observe(
      {
        stampSeconds(message->header.stamp),
        stampSeconds(message->time_ref),
        pps_uncertainty_s_,
        true});
  }

  void observeDeviceReference(
    const sensor_msgs::msg::TimeReference & message,
    TimeSyncEstimator & estimator)
  {
    estimator.observe(
      {
        stampSeconds(message.time_ref),
        stampSeconds(message.header.stamp),
        device_reference_uncertainty_s_,
        device_reference_locked_});
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const double source_stamp_s = stampSeconds(message->header.stamp);
    imu_buffer_->add(
      {
        source_stamp_s,
        {message->linear_acceleration.x, message->linear_acceleration.y,
          message->linear_acceleration.z},
        {message->angular_velocity.x, message->angular_velocity.y,
          message->angular_velocity.z}});
    last_imu_mapping_ = mapSensorTime(
      source_stamp_s, imu_stamp_domain_ == "device", *imu_to_ros_);
  }

  void onLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    const double source_stamp_s = lidar_stamp_domain_ == "device" ?
      static_cast<double>(message->timebase) * 1.0e-9 : stampSeconds(message->header.stamp);
    if (!std::isfinite(source_stamp_s)) {
      ++lidar_nonfinite_;
      return;
    }
    if (last_lidar_stamp_s_.has_value()) {
      const double delta_s = source_stamp_s - *last_lidar_stamp_s_;
      if (delta_s == 0.0) {
        ++lidar_duplicates_;
        return;
      }
      if (delta_s < 0.0) {
        ++lidar_time_reversals_;
      } else if (delta_s > lidar_max_gap_s_) {
        ++lidar_gaps_;
      }
    }
    last_lidar_stamp_s_ = source_stamp_s;
    ++lidar_samples_;
    last_lidar_mapping_ = mapSensorTime(
      source_stamp_s, lidar_stamp_domain_ == "device", *lidar_to_ros_);
  }

  std::optional<TimeMapping> mapSensorTime(
    const double source_stamp_s,
    const bool device_domain,
    TimeSyncEstimator & device_to_ros)
  {
    const double now_ros_s = now().seconds();
    if (!device_domain) {
      return ros_to_gnss_->map(source_stamp_s, now_ros_s);
    }
    const auto ros_mapping = device_to_ros.map(source_stamp_s, source_stamp_s);
    if (!ros_mapping.has_value()) {
      return std::nullopt;
    }
    auto gnss_mapping = ros_to_gnss_->map(ros_mapping->mapped_time_s, now_ros_s);
    if (!gnss_mapping.has_value()) {
      return std::nullopt;
    }
    gnss_mapping->source_time_s = source_stamp_s;
    gnss_mapping->state = weakerState(ros_mapping->state, gnss_mapping->state);
    gnss_mapping->uncertainty_s =
      std::hypot(ros_mapping->uncertainty_s, gnss_mapping->uncertainty_s);
    return gnss_mapping;
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fgo_gil/time_sync";
    status.hardware_id = "um982_mid360";

    const double now_ros_s = now().seconds();
    const auto clock_mapping = ros_to_gnss_->map(now_ros_s, now_ros_s);
    const TimeSyncState sync_state = clock_mapping.has_value() ?
      clock_mapping->state : TimeSyncState::Unsynced;
    const auto & imu_diagnostics = imu_buffer_->diagnostics();
    const bool invalid_sensor_input = imu_diagnostics.nonfinite != 0U ||
      imu_diagnostics.time_reversals != 0U || lidar_nonfinite_ != 0U ||
      lidar_time_reversals_ != 0U;
    const bool sensor_gap = imu_diagnostics.gaps != 0U || lidar_gaps_ != 0U;
    if (sync_state == TimeSyncState::Unsynced) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "UNSYNCED";
    } else if (invalid_sensor_input) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "SENSOR_TIME_OR_VALUE_INVALID";
    } else if (sensor_gap) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "SENSOR_GAP";
    } else if (sync_state == TimeSyncState::PpsLocked) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "PPS_LOCKED";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "COARSE_NO_PPS";
    }
    status.values.push_back(keyValue("state", toString(sync_state)));
    if (clock_mapping.has_value()) {
      status.values.push_back(numericKeyValue("ros_to_gnss_offset_s", clock_mapping->offset_s));
      status.values.push_back(numericKeyValue("clock_jitter_s", clock_mapping->jitter_s));
      status.values.push_back(
        numericKeyValue("clock_uncertainty_s", clock_mapping->uncertainty_s));
    }
    appendSensorMapping(status, "imu", last_imu_mapping_);
    appendSensorMapping(status, "lidar", last_lidar_mapping_);
    status.values.push_back(numericKeyValue("imu_buffer_size", imu_buffer_->size()));
    status.values.push_back(numericKeyValue("imu_segment_id", imu_diagnostics.segment_id));
    status.values.push_back(numericKeyValue("imu_duplicates", imu_diagnostics.duplicates));
    status.values.push_back(
      numericKeyValue("imu_time_reversals", imu_diagnostics.time_reversals));
    status.values.push_back(numericKeyValue("imu_gaps", imu_diagnostics.gaps));
    status.values.push_back(numericKeyValue("imu_nonfinite", imu_diagnostics.nonfinite));
    status.values.push_back(numericKeyValue("lidar_samples", lidar_samples_));
    status.values.push_back(numericKeyValue("lidar_duplicates", lidar_duplicates_));
    status.values.push_back(numericKeyValue("lidar_time_reversals", lidar_time_reversals_));
    status.values.push_back(numericKeyValue("lidar_gaps", lidar_gaps_));
    status.values.push_back(numericKeyValue("lidar_nonfinite", lidar_nonfinite_));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
    status_pub_->publish(std::move(array));
  }

  void appendSensorMapping(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & prefix,
    const std::optional<TimeMapping> & mapping) const
  {
    status.values.push_back(
      keyValue(
        prefix + "_state", mapping.has_value() ? toString(mapping->state) : "UNMAPPED"));
    if (mapping.has_value()) {
      status.values.push_back(
        numericKeyValue(prefix + "_gnss_time_s", mapping->mapped_time_s));
      status.values.push_back(
        numericKeyValue(prefix + "_time_uncertainty_s", mapping->uncertainty_s));
    }
  }

  std::string imu_topic_;
  std::string lidar_topic_;
  std::string gnss_epoch_topic_;
  std::string gnss_pps_topic_;
  std::string imu_time_reference_topic_;
  std::string lidar_time_reference_topic_;
  std::string diagnostics_topic_;
  std::string status_topic_;
  std::string imu_stamp_domain_;
  std::string lidar_stamp_domain_;
  double gnss_reception_uncertainty_s_ = 0.02;
  double pps_uncertainty_s_ = 1.0e-4;
  double device_reference_uncertainty_s_ = 1.0e-3;
  bool device_reference_locked_ = true;
  double lidar_max_gap_s_ = 0.5;
  double diagnostics_period_s_ = 1.0;

  std::unique_ptr<TimeSyncEstimator> ros_to_gnss_;
  std::unique_ptr<TimeSyncEstimator> imu_to_ros_;
  std::unique_ptr<TimeSyncEstimator> lidar_to_ros_;
  std::unique_ptr<GnssTimeTracker> gnss_time_tracker_;
  std::unique_ptr<ImuSegmentBuffer> imu_buffer_;
  std::optional<TimeMapping> last_imu_mapping_;
  std::optional<TimeMapping> last_lidar_mapping_;
  std::optional<double> last_lidar_stamp_s_;
  std::uint64_t lidar_samples_ = 0;
  std::uint64_t lidar_duplicates_ = 0;
  std::uint64_t lidar_time_reversals_ = 0;
  std::uint64_t lidar_gaps_ = 0;
  std::uint64_t lidar_nonfinite_ = 0;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr gnss_epoch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr pps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr imu_time_reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr lidar_time_reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace fgo_gil_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fgo_gil_localizer::TimeSyncNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fgo_gil_time_sync"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
