#include "fgo_gil_localizer/float_fixed_lag_smoother.hpp"
#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/integer_ambiguity_resolver.hpp"
#include "fgo_gil_localizer/satellite_propagator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "fgo_gil_msgs/msg/lidar_constraint_batch.hpp"
#include "gnss_raw_msgs/msg/ephemeris.hpp"
#include "gnss_raw_msgs/msg/observation_epoch.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

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

Vec3 point(const geometry_msgs::msg::Point & value)
{
  return {value.x, value.y, value.z};
}

Vec3 vector(const geometry_msgs::msg::Vector3 & value)
{
  return {value.x, value.y, value.z};
}

RigidPose pose(const geometry_msgs::msg::Pose & value)
{
  return {
    Quaternion{
      value.orientation.w, value.orientation.x, value.orientation.y,
      value.orientation.z}.normalized(),
    {value.position.x, value.position.y, value.position.z}};
}

double absoluteGnssSeconds(const GnssTime & time)
{
  return static_cast<double>(time.week) * kGnssWeekSeconds + time.tow_s;
}

GnssConstellation constellation(const std::uint8_t value)
{
  if (value > static_cast<std::uint8_t>(GnssConstellation::Irnss)) {
    throw std::invalid_argument("GNSS constellation value is unsupported");
  }
  return static_cast<GnssConstellation>(value);
}

GnssObservationEpoch observationEpoch(const gnss_raw_msgs::msg::ObservationEpoch & source)
{
  GnssObservationEpoch output;
  if (source.receiver < static_cast<std::uint8_t>(GnssReceiver::Master) ||
    source.receiver > static_cast<std::uint8_t>(GnssReceiver::Base))
  {
    throw std::invalid_argument("GNSS receiver value is unsupported");
  }
  output.receiver = static_cast<GnssReceiver>(source.receiver);
  output.time = {source.week, static_cast<double>(source.milliseconds_of_week) * 1.0e-3};
  output.observations.reserve(source.observations.size());
  for (const auto & input : source.observations) {
    GnssObservation observation;
    observation.satellite = {constellation(input.constellation), input.prn};
    observation.signal = {
      observation.satellite.constellation, input.signal_type, input.l2c_signal,
      input.glonass_frequency_channel};
    observation.channel_number = input.channel_number;
    observation.pseudorange_m = input.pseudorange_m;
    observation.carrier_phase_cycles = input.carrier_phase_cycles;
    observation.doppler_hz = input.doppler_hz;
    observation.pseudorange_std_m = input.pseudorange_std_m;
    observation.carrier_phase_std_cycles = input.carrier_phase_std_cycles;
    observation.cn0_db_hz = input.cn0_db_hz;
    observation.lock_time_s = input.lock_time_s;
    observation.pseudorange_valid = input.pseudorange_valid;
    observation.carrier_phase_valid = input.carrier_phase_valid;
    observation.tracking_status = input.tracking_status;
    output.observations.push_back(observation);
  }
  return output;
}

BroadcastEphemeris ephemeris(const gnss_raw_msgs::msg::Ephemeris & source)
{
  BroadcastEphemeris output;
  output.satellite = {constellation(source.constellation), source.prn};
  output.model = static_cast<EphemerisModel>(source.model);
  output.health = source.health;
  output.week = source.week;
  output.toe_s = source.toe_s;
  output.toc_s = source.toc_s;
  output.semi_major_axis_m = source.semi_major_axis_m;
  output.delta_mean_motion_rad_s = source.delta_mean_motion_rad_s;
  output.mean_anomaly_rad = source.mean_anomaly_rad;
  output.eccentricity = source.eccentricity;
  output.argument_of_perigee_rad = source.argument_of_perigee_rad;
  output.cuc_rad = source.cuc_rad;
  output.cus_rad = source.cus_rad;
  output.crc_m = source.crc_m;
  output.crs_m = source.crs_m;
  output.cic_rad = source.cic_rad;
  output.cis_rad = source.cis_rad;
  output.inclination_rad = source.inclination_rad;
  output.inclination_rate_rad_s = source.inclination_rate_rad_s;
  output.ascending_node_rad = source.ascending_node_rad;
  output.ascending_node_rate_rad_s = source.ascending_node_rate_rad_s;
  output.clock_bias_s = source.clock_bias_s;
  output.clock_drift_s_s = source.clock_drift_s_s;
  output.clock_drift_rate_s_s2 = source.clock_drift_rate_s_s2;
  output.corrected_mean_motion_rad_s = source.corrected_mean_motion_rad_s;
  output.glonass_frequency_channel = source.glonass_frequency_channel;
  output.position_ecef_m = {
    source.position_ecef_m[0], source.position_ecef_m[1], source.position_ecef_m[2]};
  output.velocity_ecef_m_s = {
    source.velocity_ecef_m_s[0], source.velocity_ecef_m_s[1],
    source.velocity_ecef_m_s[2]};
  output.acceleration_ecef_m_s2 = {
    source.acceleration_ecef_m_s2[0], source.acceleration_ecef_m_s2[1],
    source.acceleration_ecef_m_s2[2]};
  output.glonass_clock_bias_s = source.glonass_clock_bias_s;
  output.glonass_relative_frequency_bias = source.glonass_relative_frequency_bias;
  return output;
}

}  // namespace

class FloatFgoNode : public rclcpp::Node
{
public:
  FloatFgoNode()
  : Node("fgo_gil_float_fgo")
  {
    declareParameters();
    readParameters();
    resetGraph();
    createInterfaces();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("topics.imu", "/livox/imu");
    declare_parameter<std::string>("topics.lidar_constraints", "/fgo_gil/lidar_constraints");
    declare_parameter<std::string>("topics.gnss_epoch", "/gnss/raw/observation_epoch");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/raw/ephemeris");
    declare_parameter<std::string>(
      "topics.time_sync_diagnostics", "/fgo_gil/time_sync_diagnostics");
    declare_parameter<std::string>("topics.odometry", "/fgo_gil/float_odom_ecef");
    declare_parameter<std::string>("topics.fixed_odometry", "/fgo_gil/fixed_odom_ecef");
    declare_parameter<std::string>("topics.diagnostics", "/fgo_gil/float_diagnostics");
    declare_parameter<std::string>("frames.ecef", "ecef");
    declare_parameter<std::string>("frames.body", "imu_link");

    declare_parameter<bool>("calibration.ecef_from_lidar_world.calibrated", false);
    declare_parameter<std::vector<double>>(
      "calibration.ecef_from_lidar_world.translation_m", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
      "calibration.ecef_from_lidar_world.rotation_wxyz", {1.0, 0.0, 0.0, 0.0});
    declare_parameter<bool>("calibration.gnss.base_ecef_calibrated", false);
    declare_parameter<std::vector<double>>(
      "calibration.gnss.base_ecef_m", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
      "calibration.gnss.master_in_imu_m", {0.0, -0.184, 0.134});
    declare_parameter<std::vector<double>>(
      "calibration.imu_lidar.translation_m", {-0.011, -0.02329, 0.04412});
    declare_parameter<std::vector<double>>(
      "calibration.imu_lidar.rotation_wxyz", {1.0, 0.0, 0.0, 0.0});

    declare_parameter<int>("buffers.imu_capacity", 8192);
    declare_parameter<double>("buffers.maximum_imu_gap_s", 0.05);
    declare_parameter<int>("buffers.pending_gnss_epochs", 128);
    declare_parameter<int>("buffers.maximum_ephemerides", 256);
    declare_parameter<double>("imu.acceleration_scale", 9.80665);
    declare_parameter<double>("time.maximum_gnss_keyframe_offset_s", 0.10);
    declare_parameter<double>("time.maximum_sync_age_s", 2.0);
    declare_parameter<double>("time.maximum_clock_uncertainty_s", 0.05);

    declare_parameter<double>("window.duration_s", 10.0);
    declare_parameter<int>("window.maximum_states", 20);
    declare_parameter<int>("optimizer.maximum_iterations", 6);
    declare_parameter<double>("optimizer.initial_damping", 1.0e-6);
    declare_parameter<double>("optimizer.convergence_delta_norm", 1.0e-6);
    declare_parameter<double>("optimizer.lidar_line_sigma_m", 0.05);
    declare_parameter<double>("optimizer.lidar_plane_sigma_m", 0.05);
    declare_parameter<double>("optimizer.lidar_huber_delta_sigma", 2.5);
    declare_parameter<double>("optimizer.gnss_code_huber_delta_sigma", 2.5);
    declare_parameter<double>("optimizer.gnss_carrier_huber_delta_sigma", 2.5);

    declare_parameter<bool>("integer_fixing.enabled", true);
    declare_parameter<bool>("integer_fixing.partial_fixing", true);
    declare_parameter<int>("integer_fixing.minimum_ambiguities", 4);
    declare_parameter<double>("integer_fixing.ratio_threshold", 3.0);
    declare_parameter<double>("integer_fixing.minimum_success_rate", 0.99);
    declare_parameter<double>("integer_fixing.maximum_squared_norm", 25.0);
    declare_parameter<double>("integer_fixing.maximum_position_correction_m", 0.50);
    declare_parameter<double>("integer_fixing.maximum_rotation_correction_rad", 0.10);
    declare_parameter<double>("integer_fixing.maximum_velocity_correction_m_s", 1.0);
    declare_parameter<double>("integer_fixing.maximum_cost_increase", 5.0);

    declare_parameter<double>("prior.position_sigma_m", 1.0);
    declare_parameter<double>("prior.rotation_sigma_rad", 0.2);
    declare_parameter<double>("prior.velocity_sigma_m_s", 1.0);
    declare_parameter<double>("prior.accelerometer_bias_sigma_m_s2", 0.1);
    declare_parameter<double>("prior.gyroscope_bias_sigma_rad_s", 0.01);
    declare_parameter<double>("imu_factor.position_sigma_m", 0.10);
    declare_parameter<double>("imu_factor.rotation_sigma_rad", 0.02);
    declare_parameter<double>("imu_factor.velocity_sigma_m_s", 0.10);
    declare_parameter<double>("imu_factor.accelerometer_bias_sigma_m_s2", 0.01);
    declare_parameter<double>("imu_factor.gyroscope_bias_sigma_rad_s", 0.001);

    declare_parameter<double>("gnss.maximum_epoch_offset_s", 0.020);
    declare_parameter<double>("gnss.minimum_elevation_deg", 10.0);
    declare_parameter<double>("gnss.minimum_cn0_db_hz", 25.0);
    declare_parameter<double>("gnss.reference_switch_margin_deg", 5.0);
    declare_parameter<double>("gnss.maximum_baseline_m", 20000.0);
    declare_parameter<double>("gnss.maximum_code_innovation_m", 30.0);
    declare_parameter<double>("gnss.maximum_kepler_age_s", 14400.0);
    declare_parameter<double>("gnss.maximum_glonass_age_s", 1800.0);
    declare_parameter<double>("gnss.arc_maximum_gap_s", 2.0);
    declare_parameter<double>("gnss.doppler_phase_threshold_cycles", 0.75);
    declare_parameter<double>("diagnostics_period_s", 1.0);
  }

  Vec3 vec3Parameter(const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain exactly 3 values");
    }
    const Vec3 value{values[0], values[1], values[2]};
    if (!finite(value)) {
      throw std::invalid_argument(name + " must be finite");
    }
    return value;
  }

  Quaternion quaternionParameter(const std::string & name) const
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 4U) {
      throw std::invalid_argument(name + " must contain exactly 4 wxyz values");
    }
    const Quaternion value{values[0], values[1], values[2], values[3]};
    const Quaternion normalized = value.normalized();
    if (!finite(normalized)) {
      throw std::invalid_argument(name + " must be a finite nonzero quaternion");
    }
    return normalized;
  }

  std::size_t positiveSizeParameter(const std::string & name) const
  {
    const std::int64_t value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  StateFactorNoise stateNoise(const std::string & prefix) const
  {
    StateFactorNoise noise;
    noise.position_m = get_parameter(prefix + ".position_sigma_m").as_double();
    noise.rotation_rad = get_parameter(prefix + ".rotation_sigma_rad").as_double();
    noise.velocity_m_s = get_parameter(prefix + ".velocity_sigma_m_s").as_double();
    noise.accelerometer_bias_m_s2 =
      get_parameter(prefix + ".accelerometer_bias_sigma_m_s2").as_double();
    noise.gyroscope_bias_rad_s =
      get_parameter(prefix + ".gyroscope_bias_sigma_rad_s").as_double();
    return noise;
  }

  void readParameters()
  {
    imu_topic_ = get_parameter("topics.imu").as_string();
    lidar_constraints_topic_ = get_parameter("topics.lidar_constraints").as_string();
    gnss_epoch_topic_ = get_parameter("topics.gnss_epoch").as_string();
    ephemeris_topic_ = get_parameter("topics.ephemeris").as_string();
    time_sync_topic_ = get_parameter("topics.time_sync_diagnostics").as_string();
    odometry_topic_ = get_parameter("topics.odometry").as_string();
    fixed_odometry_topic_ = get_parameter("topics.fixed_odometry").as_string();
    diagnostics_topic_ = get_parameter("topics.diagnostics").as_string();
    ecef_frame_ = get_parameter("frames.ecef").as_string();
    body_frame_ = get_parameter("frames.body").as_string();
    ecef_world_calibrated_ =
      get_parameter("calibration.ecef_from_lidar_world.calibrated").as_bool();
    ecef_world_ = {
      quaternionParameter("calibration.ecef_from_lidar_world.rotation_wxyz"),
      vec3Parameter("calibration.ecef_from_lidar_world.translation_m")};
    base_ecef_calibrated_ = get_parameter("calibration.gnss.base_ecef_calibrated").as_bool();
    base_ecef_m_ = vec3Parameter("calibration.gnss.base_ecef_m");
    master_in_imu_m_ = vec3Parameter("calibration.gnss.master_in_imu_m");
    imu_lidar_ = {
      quaternionParameter("calibration.imu_lidar.rotation_wxyz"),
      vec3Parameter("calibration.imu_lidar.translation_m")};
    pending_gnss_capacity_ = positiveSizeParameter("buffers.pending_gnss_epochs");
    maximum_ephemerides_ = positiveSizeParameter("buffers.maximum_ephemerides");
    acceleration_scale_ = get_parameter("imu.acceleration_scale").as_double();
    maximum_gnss_keyframe_offset_s_ =
      get_parameter("time.maximum_gnss_keyframe_offset_s").as_double();
    maximum_sync_age_s_ = get_parameter("time.maximum_sync_age_s").as_double();
    maximum_clock_uncertainty_s_ =
      get_parameter("time.maximum_clock_uncertainty_s").as_double();
    diagnostics_period_s_ = get_parameter("diagnostics_period_s").as_double();

    smoother_config_.duration_s = get_parameter("window.duration_s").as_double();
    smoother_config_.maximum_states = positiveSizeParameter("window.maximum_states");
    smoother_config_.maximum_iterations = positiveSizeParameter("optimizer.maximum_iterations");
    smoother_config_.initial_damping = get_parameter("optimizer.initial_damping").as_double();
    smoother_config_.convergence_delta_norm =
      get_parameter("optimizer.convergence_delta_norm").as_double();
    lidar_factor_config_.line_sigma_m =
      get_parameter("optimizer.lidar_line_sigma_m").as_double();
    lidar_factor_config_.plane_sigma_m =
      get_parameter("optimizer.lidar_plane_sigma_m").as_double();
    lidar_factor_config_.huber_delta_sigma =
      get_parameter("optimizer.lidar_huber_delta_sigma").as_double();
    gnss_factor_config_.code_huber_delta_sigma =
      get_parameter("optimizer.gnss_code_huber_delta_sigma").as_double();
    gnss_factor_config_.carrier_huber_delta_sigma =
      get_parameter("optimizer.gnss_carrier_huber_delta_sigma").as_double();
    integer_resolver_config_.enabled = get_parameter("integer_fixing.enabled").as_bool();
    integer_resolver_config_.partial_fixing =
      get_parameter("integer_fixing.partial_fixing").as_bool();
    integer_resolver_config_.minimum_ambiguities =
      positiveSizeParameter("integer_fixing.minimum_ambiguities");
    integer_resolver_config_.ratio_threshold =
      get_parameter("integer_fixing.ratio_threshold").as_double();
    integer_resolver_config_.minimum_success_rate =
      get_parameter("integer_fixing.minimum_success_rate").as_double();
    integer_resolver_config_.maximum_squared_norm =
      get_parameter("integer_fixing.maximum_squared_norm").as_double();
    fixed_back_substitution_config_.maximum_position_correction_m =
      get_parameter("integer_fixing.maximum_position_correction_m").as_double();
    fixed_back_substitution_config_.maximum_rotation_correction_rad =
      get_parameter("integer_fixing.maximum_rotation_correction_rad").as_double();
    fixed_back_substitution_config_.maximum_velocity_correction_m_s =
      get_parameter("integer_fixing.maximum_velocity_correction_m_s").as_double();
    fixed_back_substitution_config_.maximum_cost_increase =
      get_parameter("integer_fixing.maximum_cost_increase").as_double();
    prior_noise_ = stateNoise("prior");
    imu_factor_config_.noise = stateNoise("imu_factor");
    imu_factor_config_.integration.maximum_step_s =
      get_parameter("buffers.maximum_imu_gap_s").as_double();

    const double degrees_to_radians = 3.14159265358979323846 / 180.0;
    EpochAlignerConfig aligner_config;
    aligner_config.maximum_time_offset_s = get_parameter("gnss.maximum_epoch_offset_s").as_double();
    aligner_config.maximum_buffered_epochs = pending_gnss_capacity_;
    epoch_aligner_ = std::make_unique<GnssEpochAligner>(aligner_config);
    DoubleDifferenceBuilderConfig dd_config;
    dd_config.minimum_elevation_rad =
      get_parameter("gnss.minimum_elevation_deg").as_double() * degrees_to_radians;
    dd_config.minimum_cn0_db_hz = get_parameter("gnss.minimum_cn0_db_hz").as_double();
    dd_config.maximum_baseline_m = get_parameter("gnss.maximum_baseline_m").as_double();
    dd_config.maximum_code_innovation_m =
      get_parameter("gnss.maximum_code_innovation_m").as_double();
    ReferenceSelectorConfig reference_config;
    reference_config.minimum_elevation_rad = dd_config.minimum_elevation_rad;
    reference_config.minimum_cn0_db_hz = dd_config.minimum_cn0_db_hz;
    reference_config.switch_margin_rad =
      get_parameter("gnss.reference_switch_margin_deg").as_double() * degrees_to_radians;
    AmbiguityArcConfig arc_config;
    arc_config.maximum_observation_gap_s =
      get_parameter("gnss.arc_maximum_gap_s").as_double();
    arc_config.doppler_phase_threshold_cycles =
      get_parameter("gnss.doppler_phase_threshold_cycles").as_double();
    dd_builder_ = std::make_unique<DoubleDifferenceBuilder>(
      dd_config, reference_config, arc_config);
    SatellitePropagationConfig propagation_config;
    propagation_config.maximum_kepler_age_s =
      get_parameter("gnss.maximum_kepler_age_s").as_double();
    propagation_config.maximum_glonass_age_s =
      get_parameter("gnss.maximum_glonass_age_s").as_double();
    satellite_propagator_ = std::make_unique<SatellitePropagator>(propagation_config);
    integer_resolver_ = std::make_unique<IntegerAmbiguityResolver>(integer_resolver_config_);
    imu_buffer_ = std::make_unique<ImuSegmentBuffer>(
      ImuBufferConfig{
        positiveSizeParameter("buffers.imu_capacity"),
        get_parameter("buffers.maximum_imu_gap_s").as_double()});

    if (!std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 ||
      !std::isfinite(maximum_gnss_keyframe_offset_s_) ||
      maximum_gnss_keyframe_offset_s_ <= 0.0 || !std::isfinite(maximum_sync_age_s_) ||
      maximum_sync_age_s_ <= 0.0 || !std::isfinite(maximum_clock_uncertainty_s_) ||
      maximum_clock_uncertainty_s_ <= 0.0 || !std::isfinite(diagnostics_period_s_) ||
      diagnostics_period_s_ <= 0.0)
    {
      throw std::invalid_argument("FGO node timing parameters are outside valid bounds");
    }
  }

  void resetGraph()
  {
    smoother_ = std::make_unique<FloatFixedLagSmoother>(
      smoother_config_, lidar_factor_config_, gnss_factor_config_);
    state_gnss_seconds_.clear();
    gnss_factor_states_.clear();
    most_recent_gnss_factor_state_.reset();
    last_state_id_.reset();
    fixed_state_.reset();
    solution_status_ = "FLOAT";
    last_integer_fix_ = {};
    last_back_substitution_ = {};
    estimator_state_ = "WAITING_FOR_LIDAR_KEYFRAME";
    ++graph_resets_;
  }

  void createInterfaces()
  {
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    fixed_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(fixed_odometry_topic_, 10);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FloatFgoNode::onImu, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<fgo_gil_msgs::msg::LidarConstraintBatch>(
      lidar_constraints_topic_, 10,
      std::bind(&FloatFgoNode::onLidarConstraints, this, std::placeholders::_1));
    gnss_sub_ = create_subscription<gnss_raw_msgs::msg::ObservationEpoch>(
      gnss_epoch_topic_, 50,
      std::bind(&FloatFgoNode::onGnssEpoch, this, std::placeholders::_1));
    ephemeris_sub_ = create_subscription<gnss_raw_msgs::msg::Ephemeris>(
      ephemeris_topic_, 50,
      std::bind(&FloatFgoNode::onEphemeris, this, std::placeholders::_1));
    time_sync_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      time_sync_topic_, 10,
      std::bind(&FloatFgoNode::onTimeSync, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_s_),
      std::bind(&FloatFgoNode::publishDiagnostics, this));
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const ImuBufferResult result = imu_buffer_->add(
      {
        stampSeconds(message->header.stamp),
        acceleration_scale_ * Vec3{
          message->linear_acceleration.x, message->linear_acceleration.y,
          message->linear_acceleration.z},
        {message->angular_velocity.x, message->angular_velocity.y,
          message->angular_velocity.z}});
    if (result == ImuBufferResult::ResetOnGap ||
      result == ImuBufferResult::ResetOnTimeReversal)
    {
      estimator_state_ = "IMU_SEGMENT_RESET";
    }
  }

  std::optional<ImuSample> interpolateImu(const double stamp_s) const
  {
    const auto & samples = imu_buffer_->samples();
    if (samples.empty() || stamp_s < samples.front().stamp_s || stamp_s > samples.back().stamp_s) {
      return std::nullopt;
    }
    const auto after = std::lower_bound(
      samples.begin(), samples.end(), stamp_s,
      [](const ImuSample & sample, const double value) {return sample.stamp_s < value;});
    if (after == samples.end()) {
      return samples.back();
    }
    if (after->stamp_s == stamp_s || after == samples.begin()) {
      ImuSample output = *after;
      output.stamp_s = stamp_s;
      return output;
    }
    const auto before = std::prev(after);
    const double duration = after->stamp_s - before->stamp_s;
    if (duration <= 0.0) {
      return std::nullopt;
    }
    const double ratio = (stamp_s - before->stamp_s) / duration;
    return ImuSample{
      stamp_s,
      before->acceleration_m_s2 +
      ratio * (after->acceleration_m_s2 - before->acceleration_m_s2),
      before->angular_velocity_rad_s +
      ratio * (after->angular_velocity_rad_s - before->angular_velocity_rad_s)};
  }

  std::optional<std::vector<ImuSample>> imuSegment(
    const double start_s,
    const double end_s) const
  {
    if (!std::isfinite(start_s) || !std::isfinite(end_s) || end_s <= start_s) {
      return std::nullopt;
    }
    const auto start = interpolateImu(start_s);
    const auto end = interpolateImu(end_s);
    if (!start.has_value() || !end.has_value()) {
      return std::nullopt;
    }
    std::vector<ImuSample> output;
    output.push_back(*start);
    for (const auto & sample : imu_buffer_->samples()) {
      if (sample.stamp_s > start_s && sample.stamp_s < end_s) {
        output.push_back(sample);
      }
    }
    output.push_back(*end);
    return output;
  }

  bool timeSyncReady() const
  {
    if (!ros_to_gnss_offset_s_.has_value() || !last_time_sync_reception_s_.has_value()) {
      return false;
    }
    const double age_s = now().seconds() - *last_time_sync_reception_s_;
    return age_s >= 0.0 && age_s <= maximum_sync_age_s_ &&
           (time_sync_state_ == "COARSE" || time_sync_state_ == "PPS_LOCKED") &&
           clock_uncertainty_s_ <= maximum_clock_uncertainty_s_;
  }

  void onTimeSync(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    for (const auto & status : message->status) {
      if (status.name != "fgo_gil/time_sync") {
        continue;
      }
      std::optional<double> offset;
      double uncertainty = std::numeric_limits<double>::infinity();
      std::string state = "UNSYNCED";
      for (const auto & value : status.values) {
        try {
          if (value.key == "state") {
            state = value.value;
          } else if (value.key == "ros_to_gnss_offset_s") {
            offset = std::stod(value.value);
          } else if (value.key == "clock_uncertainty_s") {
            uncertainty = std::stod(value.value);
          }
        } catch (const std::exception &) {
          offset.reset();
        }
      }
      time_sync_state_ = state;
      ros_to_gnss_offset_s_ = offset;
      clock_uncertainty_s_ = uncertainty;
      last_time_sync_reception_s_ = now().seconds();
      return;
    }
  }

  void onEphemeris(const gnss_raw_msgs::msg::Ephemeris::SharedPtr message)
  {
    try {
      const BroadcastEphemeris value = ephemeris(*message);
      ephemerides_[value.satellite] = value;
      ephemeris_order_.erase(
        std::remove(ephemeris_order_.begin(), ephemeris_order_.end(), value.satellite),
        ephemeris_order_.end());
      ephemeris_order_.push_back(value.satellite);
      while (ephemeris_order_.size() > maximum_ephemerides_) {
        const SatelliteId oldest = ephemeris_order_.front();
        ephemeris_order_.pop_front();
        ephemerides_.erase(oldest);
      }
      ++ephemerides_received_;
    } catch (const std::exception &) {
      ++invalid_ephemerides_;
    }
  }

  void onGnssEpoch(const gnss_raw_msgs::msg::ObservationEpoch::SharedPtr message)
  {
    try {
      const auto aligned = epoch_aligner_->push(observationEpoch(*message));
      if (!aligned.has_value()) {
        return;
      }
      ++aligned_gnss_epochs_;
      if (!attachToClosestState(*aligned)) {
        pending_gnss_.push_back(*aligned);
        while (pending_gnss_.size() > pending_gnss_capacity_) {
          pending_gnss_.pop_front();
          ++dropped_pending_gnss_;
        }
      } else if (smoother_->optimize()) {
        pruneStateBookkeeping();
        updateIntegerSolution();
        estimator_state_ = solution_status_ == "FIXED" ? "FIXED_ACTIVE" : "FLOAT_ACTIVE";
        publishOdometry();
      } else {
        estimator_state_ = "OPTIMIZATION_FAILED";
        ++optimization_failures_;
      }
    } catch (const std::exception &) {
      ++invalid_gnss_epochs_;
    }
  }

  bool attachToClosestState(const AlignedGnssEpochs & epochs)
  {
    if (!base_ecef_calibrated_ || !timeSyncReady() || state_gnss_seconds_.empty()) {
      return false;
    }
    const double epoch_seconds = absoluteGnssSeconds(epochs.rover.time);
    StateId closest_id = 0;
    double closest_offset = std::numeric_limits<double>::infinity();
    for (const auto & state_time : state_gnss_seconds_) {
      if (smoother_->state(state_time.first) == nullptr) {
        continue;
      }
      const double offset = std::abs(state_time.second - epoch_seconds);
      if (offset < closest_offset) {
        closest_id = state_time.first;
        closest_offset = offset;
      }
    }
    if (closest_offset > maximum_gnss_keyframe_offset_s_) {
      return false;
    }
    if (gnss_factor_states_.find(closest_id) != gnss_factor_states_.end()) {
      if (!most_recent_gnss_factor_state_.has_value() ||
        closest_id > *most_recent_gnss_factor_state_)
      {
        most_recent_gnss_factor_state_ = closest_id;
      }
      return true;
    }
    const EcefState * current_state = smoother_->state(closest_id);
    if (current_state == nullptr) {
      return false;
    }
    SatelliteStateMap satellite_states;
    for (const auto & observation : epochs.rover.observations) {
      if (!observation.pseudorange_valid ||
        satellite_states.find(observation.satellite) != satellite_states.end())
      {
        continue;
      }
      const auto broadcast = ephemerides_.find(observation.satellite);
      if (broadcast == ephemerides_.end()) {
        continue;
      }
      const auto propagated = satellite_propagator_->propagateToReceiveFrame(
        broadcast->second, epochs.rover.time, observation.pseudorange_m);
      if (propagated.ok()) {
        satellite_states[observation.satellite] = *propagated.state;
      } else {
        ++satellite_propagation_failures_;
      }
    }
    const auto measurements = dd_builder_->build(
      epochs, satellite_states, *current_state, base_ecef_m_, master_in_imu_m_);
    if (measurements.empty()) {
      smoother_->recordGnssOutage();
      ++gnss_rejected_epochs_;
      return true;
    }
    if (!smoother_->addGnssFactors(closest_id, measurements)) {
      ++gnss_rejected_epochs_;
      return true;
    }
    gnss_factor_states_.insert(closest_id);
    if (!most_recent_gnss_factor_state_.has_value() ||
      closest_id > *most_recent_gnss_factor_state_)
    {
      most_recent_gnss_factor_state_ = closest_id;
    }
    gnss_measurements_ += measurements.size();
    return true;
  }

  void onLidarConstraints(const fgo_gil_msgs::msg::LidarConstraintBatch::SharedPtr message)
  {
    if (!message->keyframe) {
      return;
    }
    if (!ecef_world_calibrated_) {
      estimator_state_ = "WAITING_FOR_CALIBRATION";
      return;
    }
    if (active_frontend_epoch_.has_value() && *active_frontend_epoch_ != message->frontend_epoch) {
      resetGraph();
      pending_gnss_.clear();
    }
    active_frontend_epoch_ = message->frontend_epoch;
    const double stamp_s = stampSeconds(message->header.stamp);
    const RigidPose world_lidar = pose(message->initial_pose_world_lidar);
    if (!std::isfinite(stamp_s) || !finite(world_lidar)) {
      ++invalid_lidar_batches_;
      return;
    }
    const RigidPose ecef_lidar = compose(ecef_world_, world_lidar);
    const RigidPose ecef_imu = compose(ecef_lidar, inverse(imu_lidar_));
    EcefState initial;
    initial.stamp_s = stamp_s;
    initial.position_ecef_m = ecef_imu.translation;
    initial.orientation_ecef_body = ecef_imu.rotation;
    if (last_state_id_.has_value()) {
      const EcefState * previous = smoother_->state(*last_state_id_);
      if (previous != nullptr) {
        initial.velocity_ecef_m_s = previous->velocity_ecef_m_s;
        initial.accelerometer_bias_m_s2 = previous->accelerometer_bias_m_s2;
        initial.gyroscope_bias_rad_s = previous->gyroscope_bias_rad_s;
      }
    }
    const StateId state_id = next_state_id_++;
    std::optional<std::vector<ImuSample>> imu_samples;
    if (last_state_id_.has_value()) {
      const EcefState * previous = smoother_->state(*last_state_id_);
      if (previous == nullptr) {
        resetGraph();
        return;
      }
      imu_samples = imuSegment(previous->stamp_s, stamp_s);
      if (!imu_samples.has_value()) {
        estimator_state_ = "WAITING_FOR_CONTINUOUS_IMU";
        ++lidar_batches_without_imu_;
        return;
      }
    }
    if (!smoother_->addState(state_id, initial)) {
      ++invalid_lidar_batches_;
      return;
    }
    if (!last_state_id_.has_value()) {
      if (!smoother_->addStatePrior(state_id, initial, prior_noise_)) {
        resetGraph();
        return;
      }
    } else if (!smoother_->addImuFactor(
        *last_state_id_, state_id, *imu_samples, imu_factor_config_))
    {
      resetGraph();
      estimator_state_ = "IMU_FACTOR_REJECTED";
      return;
    }

    std::vector<PointToLineFactor> lines;
    lines.reserve(message->line_factors.size());
    for (const auto & source : message->line_factors) {
      lines.push_back(
        {
          point(source.point_lidar), transformPoint(ecef_world_, point(source.line_anchor_world)),
          ecef_world_.rotation.rotate(vector(source.line_direction_world))});
    }
    std::vector<PointToPlaneFactor> planes;
    planes.reserve(message->plane_factors.size());
    for (const auto & source : message->plane_factors) {
      planes.push_back(
        {
          point(source.point_lidar), transformPoint(ecef_world_, point(source.plane_anchor_world)),
          ecef_world_.rotation.rotate(vector(source.plane_normal_world))});
    }
    if (!message->initialization_keyframe && !lines.empty() && !planes.empty()) {
      if (!smoother_->addLidarFactors(state_id, lines, planes, imu_lidar_)) {
        resetGraph();
        return;
      }
    }
    last_state_id_ = state_id;
    if (timeSyncReady()) {
      state_gnss_seconds_[state_id] = stamp_s + *ros_to_gnss_offset_s_;
      attachPendingGnss(state_id);
    } else {
      smoother_->recordGnssOutage();
    }
    if (!smoother_->optimize()) {
      estimator_state_ = "OPTIMIZATION_FAILED";
      ++optimization_failures_;
      return;
    }
    pruneStateBookkeeping();
    if (base_ecef_calibrated_) {
      updateIntegerSolution();
      estimator_state_ = solution_status_ == "FIXED" ? "FIXED_ACTIVE" : "FLOAT_ACTIVE";
    } else {
      fixed_state_.reset();
      solution_status_ = "FLOAT";
      estimator_state_ = "LIO_ONLY_WAITING_BASE";
    }
    ++lidar_keyframes_;
    publishOdometry();
  }

  void updateIntegerSolution()
  {
    fixed_state_.reset();
    solution_status_ = "FLOAT";
    last_back_substitution_ = {};
    if (!last_state_id_.has_value() || !most_recent_gnss_factor_state_.has_value() ||
      *most_recent_gnss_factor_state_ != *last_state_id_)
    {
      last_integer_fix_ = {};
      last_integer_fix_.rejection_reason = IntegerFixRejectionReason::NoCurrentGnssEpoch;
      ++integer_fix_rejections_;
      return;
    }
    const auto estimate = smoother_->floatAmbiguityEstimate();
    if (!estimate.has_value()) {
      last_integer_fix_ = {};
      last_integer_fix_.rejection_reason =
        IntegerFixRejectionReason::CovarianceNotPositiveDefinite;
      ++integer_fix_rejections_;
      return;
    }
    last_integer_fix_ = integer_resolver_->resolve(*estimate);
    if (!last_integer_fix_.fixed) {
      ++integer_fix_rejections_;
      return;
    }
    last_back_substitution_ = smoother_->previewFixedAmbiguities(
      last_integer_fix_.keys, last_integer_fix_.fixed_values_m,
      fixed_back_substitution_config_);
    if (!last_back_substitution_.accepted) {
      last_integer_fix_.fixed = false;
      last_integer_fix_.rejection_reason =
        IntegerFixRejectionReason::BackSubstitutionRejected;
      ++integer_fix_rejections_;
      return;
    }
    fixed_state_ = last_back_substitution_.latest_state;
    solution_status_ = "FIXED";
    ++integer_fixed_solutions_;
  }

  void attachPendingGnss(const StateId state_id)
  {
    const auto state_time = state_gnss_seconds_.find(state_id);
    if (state_time == state_gnss_seconds_.end()) {
      return;
    }
    auto closest = pending_gnss_.end();
    double closest_offset = std::numeric_limits<double>::infinity();
    for (auto iterator = pending_gnss_.begin(); iterator != pending_gnss_.end(); ++iterator) {
      const double offset = std::abs(
        absoluteGnssSeconds(iterator->rover.time) - state_time->second);
      if (offset < closest_offset) {
        closest = iterator;
        closest_offset = offset;
      }
    }
    if (closest != pending_gnss_.end() && closest_offset <= maximum_gnss_keyframe_offset_s_) {
      const AlignedGnssEpochs epochs = *closest;
      pending_gnss_.erase(closest);
      attachToClosestState(epochs);
    } else {
      smoother_->recordGnssOutage();
    }
  }

  void publishOdometry()
  {
    if (!last_state_id_.has_value()) {
      return;
    }
    const EcefState * state = smoother_->state(*last_state_id_);
    if (state == nullptr) {
      return;
    }
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(
      static_cast<std::int64_t>(std::llround(state->stamp_s * 1.0e9)));
    message.header.frame_id = ecef_frame_;
    message.child_frame_id = body_frame_;
    message.pose.pose.position.x = state->position_ecef_m.x;
    message.pose.pose.position.y = state->position_ecef_m.y;
    message.pose.pose.position.z = state->position_ecef_m.z;
    message.pose.pose.orientation.w = state->orientation_ecef_body.w;
    message.pose.pose.orientation.x = state->orientation_ecef_body.x;
    message.pose.pose.orientation.y = state->orientation_ecef_body.y;
    message.pose.pose.orientation.z = state->orientation_ecef_body.z;
    message.twist.twist.linear.x = state->velocity_ecef_m_s.x;
    message.twist.twist.linear.y = state->velocity_ecef_m_s.y;
    message.twist.twist.linear.z = state->velocity_ecef_m_s.z;
    odometry_pub_->publish(std::move(message));
    if (!fixed_state_.has_value()) {
      return;
    }
    nav_msgs::msg::Odometry fixed_message;
    fixed_message.header.stamp = rclcpp::Time(
      static_cast<std::int64_t>(std::llround(fixed_state_->stamp_s * 1.0e9)));
    fixed_message.header.frame_id = ecef_frame_;
    fixed_message.child_frame_id = body_frame_;
    fixed_message.pose.pose.position.x = fixed_state_->position_ecef_m.x;
    fixed_message.pose.pose.position.y = fixed_state_->position_ecef_m.y;
    fixed_message.pose.pose.position.z = fixed_state_->position_ecef_m.z;
    fixed_message.pose.pose.orientation.w = fixed_state_->orientation_ecef_body.w;
    fixed_message.pose.pose.orientation.x = fixed_state_->orientation_ecef_body.x;
    fixed_message.pose.pose.orientation.y = fixed_state_->orientation_ecef_body.y;
    fixed_message.pose.pose.orientation.z = fixed_state_->orientation_ecef_body.z;
    fixed_message.twist.twist.linear.x = fixed_state_->velocity_ecef_m_s.x;
    fixed_message.twist.twist.linear.y = fixed_state_->velocity_ecef_m_s.y;
    fixed_message.twist.twist.linear.z = fixed_state_->velocity_ecef_m_s.z;
    fixed_odometry_pub_->publish(std::move(fixed_message));
  }

  void pruneStateBookkeeping()
  {
    for (auto iterator = state_gnss_seconds_.begin(); iterator != state_gnss_seconds_.end(); ) {
      if (smoother_->state(iterator->first) == nullptr) {
        if (most_recent_gnss_factor_state_ == iterator->first) {
          most_recent_gnss_factor_state_.reset();
        }
        gnss_factor_states_.erase(iterator->first);
        iterator = state_gnss_seconds_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fgo_gil/float_fgo";
    status.hardware_id = "jetson_orin_nx";
    status.message = estimator_state_;
    if (estimator_state_ == "FLOAT_ACTIVE" || estimator_state_ == "FIXED_ACTIVE") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (estimator_state_ == "OPTIMIZATION_FAILED" ||
      estimator_state_ == "IMU_FACTOR_REJECTED")
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    const auto & graph = smoother_->diagnostics();
    const auto & dd = dd_builder_->diagnostics();
    status.values.push_back(keyValue("time_sync_state", time_sync_state_));
    status.values.push_back(
      keyValue("ecef_world_calibrated", ecef_world_calibrated_ ? "true" : "false"));
    status.values.push_back(
      keyValue("base_ecef_calibrated", base_ecef_calibrated_ ? "true" : "false"));
    status.values.push_back(keyValue("solution_status", solution_status_));
    status.values.push_back(
      keyValue("fix_rejection_reason", toString(last_integer_fix_.rejection_reason)));
    status.values.push_back(
      keyValue("back_substitution_rejection", toString(last_back_substitution_.rejection)));
    status.values.push_back(numericKeyValue("ambiguity_ratio", last_integer_fix_.ratio));
    status.values.push_back(
      numericKeyValue("ambiguity_success_rate", last_integer_fix_.success_rate));
    status.values.push_back(
      numericKeyValue("ambiguity_best_squared_norm", last_integer_fix_.best_squared_norm));
    status.values.push_back(numericKeyValue(
      "fixed_ambiguities", last_integer_fix_.fixed ? last_integer_fix_.keys.size() : 0U));
    status.values.push_back(numericKeyValue(
      "candidate_ambiguities", last_integer_fix_.keys.size()));
    status.values.push_back(
      numericKeyValue("integer_fixed_solutions", integer_fixed_solutions_));
    status.values.push_back(
      numericKeyValue("integer_fix_rejections", integer_fix_rejections_));
    status.values.push_back(numericKeyValue(
      "fixed_position_correction_m", last_back_substitution_.maximum_position_correction_m));
    status.values.push_back(numericKeyValue(
      "fixed_rotation_correction_rad", last_back_substitution_.maximum_rotation_correction_rad));
    status.values.push_back(numericKeyValue(
      "fixed_cost_increase", last_back_substitution_.cost_after -
      last_back_substitution_.cost_before));
    status.values.push_back(numericKeyValue("clock_uncertainty_s", clock_uncertainty_s_));
    status.values.push_back(numericKeyValue("states", graph.states));
    status.values.push_back(numericKeyValue("ambiguities", graph.ambiguities));
    status.values.push_back(numericKeyValue("factors", graph.factors));
    status.values.push_back(numericKeyValue("marginalizations", graph.marginalizations));
    status.values.push_back(numericKeyValue("gnss_outages", graph.gnss_outages));
    status.values.push_back(numericKeyValue("last_iterations", graph.last_iterations));
    status.values.push_back(numericKeyValue("last_cost", graph.last_cost));
    status.values.push_back(
      numericKeyValue("last_condition_estimate", graph.last_condition_estimate));
    status.values.push_back(numericKeyValue("lidar_keyframes", lidar_keyframes_));
    status.values.push_back(numericKeyValue("aligned_gnss_epochs", aligned_gnss_epochs_));
    status.values.push_back(numericKeyValue("gnss_measurements", gnss_measurements_));
    status.values.push_back(numericKeyValue("dd_code_factors", dd.code_factors));
    status.values.push_back(numericKeyValue("dd_carrier_factors", dd.carrier_factors));
    status.values.push_back(numericKeyValue("dd_new_arcs", dd.new_arcs));
    status.values.push_back(numericKeyValue("ephemerides", ephemerides_.size()));
    status.values.push_back(numericKeyValue("pending_gnss", pending_gnss_.size()));
    status.values.push_back(
      numericKeyValue("satellite_propagation_failures", satellite_propagation_failures_));
    status.values.push_back(numericKeyValue("gnss_rejected_epochs", gnss_rejected_epochs_));
    status.values.push_back(numericKeyValue("optimization_failures", optimization_failures_));
    status.values.push_back(numericKeyValue("graph_resets", graph_resets_));
    const auto & imu = imu_buffer_->diagnostics();
    status.values.push_back(numericKeyValue("imu_segment_id", imu.segment_id));
    status.values.push_back(numericKeyValue("imu_gaps", imu.gaps));
    status.values.push_back(numericKeyValue("imu_time_reversals", imu.time_reversals));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string imu_topic_;
  std::string lidar_constraints_topic_;
  std::string gnss_epoch_topic_;
  std::string ephemeris_topic_;
  std::string time_sync_topic_;
  std::string odometry_topic_;
  std::string fixed_odometry_topic_;
  std::string diagnostics_topic_;
  std::string ecef_frame_;
  std::string body_frame_;
  bool ecef_world_calibrated_ = false;
  bool base_ecef_calibrated_ = false;
  RigidPose ecef_world_;
  RigidPose imu_lidar_;
  Vec3 base_ecef_m_;
  Vec3 master_in_imu_m_;
  std::size_t pending_gnss_capacity_ = 128;
  std::size_t maximum_ephemerides_ = 256;
  double acceleration_scale_ = 9.80665;
  double maximum_gnss_keyframe_offset_s_ = 0.10;
  double maximum_sync_age_s_ = 2.0;
  double maximum_clock_uncertainty_s_ = 0.05;
  double diagnostics_period_s_ = 1.0;
  FloatSmootherConfig smoother_config_;
  LidarGraphFactorConfig lidar_factor_config_;
  GnssGraphFactorConfig gnss_factor_config_;
  IntegerAmbiguityResolverConfig integer_resolver_config_;
  FixedBackSubstitutionConfig fixed_back_substitution_config_;
  StateFactorNoise prior_noise_;
  ImuGraphFactorConfig imu_factor_config_;

  std::unique_ptr<ImuSegmentBuffer> imu_buffer_;
  std::unique_ptr<GnssEpochAligner> epoch_aligner_;
  std::unique_ptr<DoubleDifferenceBuilder> dd_builder_;
  std::unique_ptr<SatellitePropagator> satellite_propagator_;
  std::unique_ptr<IntegerAmbiguityResolver> integer_resolver_;
  std::unique_ptr<FloatFixedLagSmoother> smoother_;
  std::map<SatelliteId, BroadcastEphemeris> ephemerides_;
  std::deque<SatelliteId> ephemeris_order_;
  std::deque<AlignedGnssEpochs> pending_gnss_;
  std::map<StateId, double> state_gnss_seconds_;
  std::set<StateId> gnss_factor_states_;
  std::optional<StateId> last_state_id_;
  std::optional<StateId> most_recent_gnss_factor_state_;
  std::optional<EcefState> fixed_state_;
  std::optional<std::uint64_t> active_frontend_epoch_;
  StateId next_state_id_ = 1;
  std::string estimator_state_ = "WAITING_FOR_INPUT";
  std::string solution_status_ = "FLOAT";
  IntegerFixResult last_integer_fix_;
  FixedBackSubstitutionResult last_back_substitution_;
  std::string time_sync_state_ = "UNSYNCED";
  std::optional<double> ros_to_gnss_offset_s_;
  std::optional<double> last_time_sync_reception_s_;
  double clock_uncertainty_s_ = std::numeric_limits<double>::infinity();

  std::uint64_t graph_resets_ = 0;
  std::uint64_t lidar_keyframes_ = 0;
  std::uint64_t lidar_batches_without_imu_ = 0;
  std::uint64_t invalid_lidar_batches_ = 0;
  std::uint64_t aligned_gnss_epochs_ = 0;
  std::uint64_t invalid_gnss_epochs_ = 0;
  std::uint64_t dropped_pending_gnss_ = 0;
  std::uint64_t gnss_rejected_epochs_ = 0;
  std::uint64_t gnss_measurements_ = 0;
  std::uint64_t ephemerides_received_ = 0;
  std::uint64_t invalid_ephemerides_ = 0;
  std::uint64_t satellite_propagation_failures_ = 0;
  std::uint64_t optimization_failures_ = 0;
  std::uint64_t integer_fixed_solutions_ = 0;
  std::uint64_t integer_fix_rejections_ = 0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fixed_odometry_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<fgo_gil_msgs::msg::LidarConstraintBatch>::SharedPtr lidar_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr gnss_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::Ephemeris>::SharedPtr ephemeris_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr time_sync_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace fgo_gil_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fgo_gil_localizer::FloatFgoNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fgo_gil_float_fgo"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
