#include "fgo_gil_localizer/float_fixed_lag_smoother.hpp"
#include "fgo_gil_localizer/imu_buffer.hpp"
#include "fgo_gil_localizer/integer_ambiguity_resolver.hpp"
#include "fgo_gil_localizer/reference_station_tracker.hpp"
#include "fgo_gil_localizer/satellite_propagator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gnss_raw_msgs/msg/ephemeris.hpp"
#include "gnss_raw_msgs/msg/observation_epoch.hpp"
#include "gnss_raw_msgs/msg/reference_station.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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

const char * constellationLabel(const GnssConstellation constellation)
{
  switch (constellation) {
    case GnssConstellation::Gps: return "GPS";
    case GnssConstellation::Glonass: return "GLONASS";
    case GnssConstellation::Galileo: return "GALILEO";
    case GnssConstellation::Bds: return "BEIDOU";
    case GnssConstellation::Qzss: return "QZSS";
    case GnssConstellation::Sbas: return "SBAS";
    case GnssConstellation::Irnss: return "IRNSS";
    default: return "UNKNOWN";
  }
}

std::string signalGroupLabel(const SignalGroup & group)
{
  return std::string(constellationLabel(group.constellation)) + "_signal_" +
         std::to_string(group.signal_type) + (group.l2c_signal ? "_l2c" : "");
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
    // UM982 OBSVM exposes OEM accumulated Doppler range, whose sign is opposite
    // to the standard positive-range carrier phase used by the DD model.
    observation.carrier_phase_cycles = um982AdrToCarrierPhaseCycles(
      input.carrier_phase_cycles);
    observation.doppler_hz = input.doppler_hz;
    // UM982 OBSVBASE uses the shared record slot but does not output base Doppler.
    observation.doppler_valid = output.receiver != GnssReceiver::Base;
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
      "topics.reference_station", "/gnss/rtcm/reference_station");
    declare_parameter<std::string>(
      "topics.time_sync_diagnostics", "/fgo_gil/time_sync_diagnostics");
    declare_parameter<std::string>("topics.odometry", "/fgo_gil/float_odom_ecef");
    declare_parameter<std::string>("topics.fixed_odometry", "/fgo_gil/fixed_odom_ecef");
    declare_parameter<std::string>("topics.output_odometry", "/fgo_gil/odom");
    declare_parameter<std::string>("topics.path", "/fgo_gil/path");
    declare_parameter<std::string>("topics.diagnostics", "/fgo_gil/float_diagnostics");
    declare_parameter<std::string>(
      "topics.factor_diagnostics", "/fgo_gil/factor_diagnostics");
    declare_parameter<std::string>("topics.ambiguity_status", "/fgo_gil/ambiguity_status");
    declare_parameter<std::string>("topics.performance", "/fgo_gil/performance");
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
    declare_parameter<bool>("calibration.gnss.dynamic_base.enabled", true);
    declare_parameter<double>("calibration.gnss.dynamic_base.change_threshold_m", 0.01);
    declare_parameter<std::vector<double>>(
      "calibration.gnss.master_in_imu_m", {0.0, -0.184, 0.134});
    declare_parameter<std::vector<double>>(
      "calibration.imu_lidar.translation_m", {-0.011, -0.02329, 0.04412});
    declare_parameter<std::vector<double>>(
      "calibration.imu_lidar.rotation_wxyz", {1.0, 0.0, 0.0, 0.0});

    declare_parameter<int>("buffers.imu_capacity", 8192);
    declare_parameter<int>("buffers.imu_qos_depth", 512);
    declare_parameter<int>("buffers.raw_input_qos_depth", 512);
    declare_parameter<double>("buffers.maximum_imu_gap_s", 0.05);
    declare_parameter<int>("buffers.pending_lidar_batches", 16);
    declare_parameter<double>("buffers.pending_lidar_timeout_s", 0.5);
    declare_parameter<int>("buffers.pending_raw_epochs", 1024);
    declare_parameter<int>("buffers.pending_ephemerides", 64);
    declare_parameter<int>("buffers.pending_reference_stations", 16);
    declare_parameter<int>("buffers.pending_gnss_epochs", 128);
    declare_parameter<int>("buffers.maximum_ephemerides", 256);
    declare_parameter<double>("imu.acceleration_scale", 9.80665);
    declare_parameter<double>("time.maximum_gnss_keyframe_offset_s", 0.10);
    declare_parameter<double>("time.maximum_sync_age_s", 2.0);
    declare_parameter<double>("time.maximum_clock_uncertainty_s", 0.05);
    declare_parameter<double>("raw_input.startup_grace_s", 5.0);
    declare_parameter<double>("raw_input.observation_stale_timeout_s", 2.0);
    declare_parameter<double>("raw_input.ephemeris_stale_timeout_s", 300.0);
    declare_parameter<int>("output.maximum_path_poses", 2000);
    declare_parameter<bool>("safety.publish_tf", false);
    declare_parameter<bool>("safety.nav2_use_fgo", false);

    declare_parameter<double>("window.duration_s", 10.0);
    declare_parameter<int>("window.maximum_states", 20);
    declare_parameter<int>("optimizer.maximum_iterations", 6);
    declare_parameter<double>("optimizer.initial_damping", 1.0e-6);
    declare_parameter<double>("optimizer.convergence_delta_norm", 1.0e-6);
    declare_parameter<double>("optimizer.maximum_condition_estimate", 1.0e12);
    declare_parameter<double>("optimizer.lidar_line_sigma_m", 0.05);
    declare_parameter<double>("optimizer.lidar_plane_sigma_m", 0.05);
    declare_parameter<double>("optimizer.lidar_huber_delta_sigma", 2.5);
    declare_parameter<int>("optimizer.maximum_line_factors_per_keyframe", 48);
    declare_parameter<int>("optimizer.maximum_plane_factors_per_keyframe", 96);
    declare_parameter<double>("optimizer.gnss_code_huber_delta_sigma", 2.5);
    declare_parameter<double>("optimizer.gnss_carrier_huber_delta_sigma", 2.5);

    declare_parameter<bool>("integer_fixing.enabled", true);
    declare_parameter<bool>("integer_fixing.partial_fixing", true);
    declare_parameter<int>("integer_fixing.minimum_ambiguities", 4);
    declare_parameter<int>("integer_fixing.minimum_observation_epochs", 5);
    declare_parameter<int>("integer_fixing.minimum_consecutive_fixes", 3);
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
    declare_parameter<double>("gnss.base_arc_maximum_gap_s", 5.0);
    declare_parameter<double>("gnss.doppler_phase_threshold_cycles", 0.75);
    declare_parameter<double>("gnss.unavailable_base_code_sigma_m", 0.30);
    declare_parameter<double>("gnss.unavailable_base_carrier_sigma_m", 0.01);
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
    reference_station_topic_ = get_parameter("topics.reference_station").as_string();
    time_sync_topic_ = get_parameter("topics.time_sync_diagnostics").as_string();
    odometry_topic_ = get_parameter("topics.odometry").as_string();
    fixed_odometry_topic_ = get_parameter("topics.fixed_odometry").as_string();
    output_odometry_topic_ = get_parameter("topics.output_odometry").as_string();
    path_topic_ = get_parameter("topics.path").as_string();
    diagnostics_topic_ = get_parameter("topics.diagnostics").as_string();
    factor_diagnostics_topic_ = get_parameter("topics.factor_diagnostics").as_string();
    ambiguity_status_topic_ = get_parameter("topics.ambiguity_status").as_string();
    performance_topic_ = get_parameter("topics.performance").as_string();
    ecef_frame_ = get_parameter("frames.ecef").as_string();
    body_frame_ = get_parameter("frames.body").as_string();
    ecef_world_calibrated_ =
      get_parameter("calibration.ecef_from_lidar_world.calibrated").as_bool();
    ecef_world_ = {
      quaternionParameter("calibration.ecef_from_lidar_world.rotation_wxyz"),
      vec3Parameter("calibration.ecef_from_lidar_world.translation_m")};
    static_base_ecef_calibrated_ =
      get_parameter("calibration.gnss.base_ecef_calibrated").as_bool();
    base_ecef_m_ = vec3Parameter("calibration.gnss.base_ecef_m");
    dynamic_base_enabled_ = get_parameter("calibration.gnss.dynamic_base.enabled").as_bool();
    reference_station_tracker_ = std::make_unique<ReferenceStationTracker>(
      ReferenceStationTrackerConfig{
        get_parameter("calibration.gnss.dynamic_base.change_threshold_m").as_double(),
        5.0e6,
        7.0e6});
    base_ecef_source_ = static_base_ecef_calibrated_ ? "STATIC_PARAMETER" : "WAITING";
    master_in_imu_m_ = vec3Parameter("calibration.gnss.master_in_imu_m");
    imu_lidar_ = {
      quaternionParameter("calibration.imu_lidar.rotation_wxyz"),
      vec3Parameter("calibration.imu_lidar.translation_m")};
    pending_gnss_capacity_ = positiveSizeParameter("buffers.pending_gnss_epochs");
    imu_qos_depth_ = positiveSizeParameter("buffers.imu_qos_depth");
    raw_input_qos_depth_ = positiveSizeParameter("buffers.raw_input_qos_depth");
    pending_lidar_capacity_ = positiveSizeParameter("buffers.pending_lidar_batches");
    pending_lidar_timeout_s_ = get_parameter("buffers.pending_lidar_timeout_s").as_double();
    pending_raw_epoch_capacity_ = positiveSizeParameter("buffers.pending_raw_epochs");
    pending_ephemeris_capacity_ = positiveSizeParameter("buffers.pending_ephemerides");
    pending_reference_station_capacity_ =
      positiveSizeParameter("buffers.pending_reference_stations");
    maximum_ephemerides_ = positiveSizeParameter("buffers.maximum_ephemerides");
    acceleration_scale_ = get_parameter("imu.acceleration_scale").as_double();
    maximum_gnss_keyframe_offset_s_ =
      get_parameter("time.maximum_gnss_keyframe_offset_s").as_double();
    maximum_sync_age_s_ = get_parameter("time.maximum_sync_age_s").as_double();
    maximum_clock_uncertainty_s_ =
      get_parameter("time.maximum_clock_uncertainty_s").as_double();
    diagnostics_period_s_ = get_parameter("diagnostics_period_s").as_double();
    raw_startup_grace_s_ = get_parameter("raw_input.startup_grace_s").as_double();
    observation_stale_timeout_s_ =
      get_parameter("raw_input.observation_stale_timeout_s").as_double();
    ephemeris_stale_timeout_s_ =
      get_parameter("raw_input.ephemeris_stale_timeout_s").as_double();
    maximum_path_poses_ = positiveSizeParameter("output.maximum_path_poses");
    publish_tf_ = get_parameter("safety.publish_tf").as_bool();
    nav2_use_fgo_ = get_parameter("safety.nav2_use_fgo").as_bool();

    smoother_config_.duration_s = get_parameter("window.duration_s").as_double();
    smoother_config_.maximum_states = positiveSizeParameter("window.maximum_states");
    smoother_config_.maximum_iterations = positiveSizeParameter("optimizer.maximum_iterations");
    smoother_config_.initial_damping = get_parameter("optimizer.initial_damping").as_double();
    smoother_config_.convergence_delta_norm =
      get_parameter("optimizer.convergence_delta_norm").as_double();
    maximum_condition_estimate_ =
      get_parameter("optimizer.maximum_condition_estimate").as_double();
    lidar_factor_config_.line_sigma_m =
      get_parameter("optimizer.lidar_line_sigma_m").as_double();
    lidar_factor_config_.plane_sigma_m =
      get_parameter("optimizer.lidar_plane_sigma_m").as_double();
    lidar_factor_config_.huber_delta_sigma =
      get_parameter("optimizer.lidar_huber_delta_sigma").as_double();
    lidar_factor_config_.maximum_line_factors_per_keyframe =
      positiveSizeParameter("optimizer.maximum_line_factors_per_keyframe");
    lidar_factor_config_.maximum_plane_factors_per_keyframe =
      positiveSizeParameter("optimizer.maximum_plane_factors_per_keyframe");
    gnss_factor_config_.code_huber_delta_sigma =
      get_parameter("optimizer.gnss_code_huber_delta_sigma").as_double();
    gnss_factor_config_.carrier_huber_delta_sigma =
      get_parameter("optimizer.gnss_carrier_huber_delta_sigma").as_double();
    integer_resolver_config_.enabled = get_parameter("integer_fixing.enabled").as_bool();
    integer_resolver_config_.partial_fixing =
      get_parameter("integer_fixing.partial_fixing").as_bool();
    integer_resolver_config_.minimum_ambiguities =
      positiveSizeParameter("integer_fixing.minimum_ambiguities");
    integer_resolver_config_.minimum_observation_epochs =
      positiveSizeParameter("integer_fixing.minimum_observation_epochs");
    minimum_consecutive_fixes_ =
      positiveSizeParameter("integer_fixing.minimum_consecutive_fixes");
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
    epoch_aligner_config_.maximum_time_offset_s =
      get_parameter("gnss.maximum_epoch_offset_s").as_double();
    epoch_aligner_config_.maximum_buffered_epochs = pending_gnss_capacity_;
    epoch_aligner_ = std::make_unique<GnssEpochAligner>(epoch_aligner_config_);
    dd_builder_config_.minimum_elevation_rad =
      get_parameter("gnss.minimum_elevation_deg").as_double() * degrees_to_radians;
    dd_builder_config_.minimum_cn0_db_hz = get_parameter("gnss.minimum_cn0_db_hz").as_double();
    dd_builder_config_.maximum_baseline_m = get_parameter("gnss.maximum_baseline_m").as_double();
    dd_builder_config_.maximum_code_innovation_m =
      get_parameter("gnss.maximum_code_innovation_m").as_double();
    dd_builder_config_.unavailable_base_code_sigma_m =
      get_parameter("gnss.unavailable_base_code_sigma_m").as_double();
    dd_builder_config_.unavailable_base_carrier_sigma_m =
      get_parameter("gnss.unavailable_base_carrier_sigma_m").as_double();
    reference_selector_config_.minimum_elevation_rad = dd_builder_config_.minimum_elevation_rad;
    reference_selector_config_.minimum_cn0_db_hz = dd_builder_config_.minimum_cn0_db_hz;
    reference_selector_config_.switch_margin_rad =
      get_parameter("gnss.reference_switch_margin_deg").as_double() * degrees_to_radians;
    ambiguity_arc_config_.maximum_observation_gap_s =
      get_parameter("gnss.arc_maximum_gap_s").as_double();
    ambiguity_arc_config_.base_maximum_observation_gap_s =
      get_parameter("gnss.base_arc_maximum_gap_s").as_double();
    ambiguity_arc_config_.doppler_phase_threshold_cycles =
      get_parameter("gnss.doppler_phase_threshold_cycles").as_double();
    dd_builder_ = std::make_unique<DoubleDifferenceBuilder>(
      dd_builder_config_, reference_selector_config_, ambiguity_arc_config_);
    SatellitePropagationConfig propagation_config;
    propagation_config.maximum_kepler_age_s =
      get_parameter("gnss.maximum_kepler_age_s").as_double();
    propagation_config.maximum_glonass_age_s =
      get_parameter("gnss.maximum_glonass_age_s").as_double();
    satellite_propagator_ = std::make_unique<SatellitePropagator>(propagation_config);
    integer_resolver_ = std::make_unique<IntegerAmbiguityResolver>(integer_resolver_config_);
    integer_confirmation_ = std::make_unique<IntegerCandidateConfirmation>(
      minimum_consecutive_fixes_);
    imu_buffer_ = std::make_unique<ImuSegmentBuffer>(
      ImuBufferConfig{
        positiveSizeParameter("buffers.imu_capacity"),
        get_parameter("buffers.maximum_imu_gap_s").as_double()});

    if (!std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 ||
      !std::isfinite(maximum_gnss_keyframe_offset_s_) ||
      maximum_gnss_keyframe_offset_s_ <= 0.0 || !std::isfinite(maximum_sync_age_s_) ||
      maximum_sync_age_s_ <= 0.0 || !std::isfinite(maximum_clock_uncertainty_s_) ||
      maximum_clock_uncertainty_s_ <= 0.0 || !std::isfinite(diagnostics_period_s_) ||
      diagnostics_period_s_ <= 0.0 || !std::isfinite(raw_startup_grace_s_) ||
      raw_startup_grace_s_ < 0.0 || !std::isfinite(observation_stale_timeout_s_) ||
      observation_stale_timeout_s_ <= 0.0 ||
      !std::isfinite(ephemeris_stale_timeout_s_) || ephemeris_stale_timeout_s_ <= 0.0 ||
      !std::isfinite(pending_lidar_timeout_s_) || pending_lidar_timeout_s_ <= 0.0 ||
      !std::isfinite(maximum_condition_estimate_) || maximum_condition_estimate_ <= 1.0)
    {
      throw std::invalid_argument("FGO node timing parameters are outside valid bounds");
    }
    if (publish_tf_ || nav2_use_fgo_) {
      throw std::invalid_argument(
              "FGO-GIL Phase 7 is shadow-only: publish_tf and nav2_use_fgo must remain false");
    }
    if (static_base_ecef_calibrated_ &&
      (!finite(base_ecef_m_) || norm(base_ecef_m_) < 5.0e6 || norm(base_ecef_m_) > 7.0e6))
    {
      throw std::invalid_argument("calibrated static GNSS base ECEF is invalid");
    }
  }

  bool baseEcefReady() const noexcept
  {
    return static_base_ecef_calibrated_ ||
           (dynamic_base_enabled_ && dynamic_base_ecef_calibrated_);
  }

  void resetGraph()
  {
    smoother_ = std::make_unique<FloatFixedLagSmoother>(
      smoother_config_, lidar_factor_config_, gnss_factor_config_);
    state_gnss_seconds_.clear();
    gnss_factor_states_.clear();
    most_recent_gnss_factor_state_.reset();
    integer_fix_pending_ = false;
    last_state_id_.reset();
    graph_imu_segment_id_.reset();
    fixed_state_.reset();
    path_msg_.poses.clear();
    path_msg_.header.frame_id = ecef_frame_;
    last_output_stamp_s_.reset();
    last_output_reception_steady_.reset();
    last_optimizer_state_stamp_s_.reset();
    solution_status_ = "FLOAT";
    last_integer_fix_ = {};
    last_back_substitution_ = {};
    if (integer_confirmation_) {
      integer_confirmation_->reset();
    }
    estimator_state_ = "WAITING_FOR_LIDAR_KEYFRAME";
    ++graph_resets_;
  }

  void createInterfaces()
  {
    imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    estimator_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    raw_input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    rclcpp::SubscriptionOptions estimator_options;
    estimator_options.callback_group = estimator_callback_group_;
    rclcpp::SubscriptionOptions raw_input_options;
    raw_input_options.callback_group = raw_input_callback_group_;

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    fixed_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(fixed_odometry_topic_, 10);
    output_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odometry_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    factor_diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(factor_diagnostics_topic_, 10);
    ambiguity_status_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(ambiguity_status_topic_, 10);
    performance_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(performance_topic_, 10);
    auto imu_qos = rclcpp::SensorDataQoS();
    imu_qos.keep_last(imu_qos_depth_);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&FloatFgoNode::onImu, this, std::placeholders::_1), imu_options);
    lidar_sub_ = create_subscription<fgo_gil_msgs::msg::LidarConstraintBatch>(
      lidar_constraints_topic_, 10,
      std::bind(&FloatFgoNode::onLidarConstraints, this, std::placeholders::_1),
      estimator_options);
    gnss_sub_ = create_subscription<gnss_raw_msgs::msg::ObservationEpoch>(
      gnss_epoch_topic_, raw_input_qos_depth_,
      std::bind(&FloatFgoNode::onGnssEpoch, this, std::placeholders::_1), raw_input_options);
    ephemeris_sub_ = create_subscription<gnss_raw_msgs::msg::Ephemeris>(
      ephemeris_topic_, raw_input_qos_depth_,
      std::bind(&FloatFgoNode::onEphemeris, this, std::placeholders::_1), raw_input_options);
    auto reference_station_qos = rclcpp::QoS(10).reliable().transient_local();
    reference_station_sub_ = create_subscription<gnss_raw_msgs::msg::ReferenceStation>(
      reference_station_topic_, reference_station_qos,
      std::bind(&FloatFgoNode::onReferenceStation, this, std::placeholders::_1),
      raw_input_options);
    time_sync_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      time_sync_topic_, 10,
      std::bind(&FloatFgoNode::onTimeSync, this, std::placeholders::_1), estimator_options);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_s_),
      std::bind(&FloatFgoNode::publishDiagnostics, this), estimator_callback_group_);
    pending_lidar_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&FloatFgoNode::drainPendingLidar, this), estimator_callback_group_);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_->add(
      {
        stampSeconds(message->header.stamp),
        acceleration_scale_ * Vec3{
          message->linear_acceleration.x, message->linear_acceleration.y,
          message->linear_acceleration.z},
        {message->angular_velocity.x, message->angular_velocity.y,
          message->angular_velocity.z}});
  }

  std::pair<ImuCoverageResult, std::uint64_t> imuCoverage(const double stamp_s) const
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return {classifyImuTime(*imu_buffer_, stamp_s), imu_buffer_->diagnostics().segment_id};
  }

  ImuSegmentSelection imuSegment(
    const double start_s,
    const double end_s) const
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return selectImuSegment(*imu_buffer_, start_s, end_s);
  }

  ImuBufferDiagnostics imuDiagnostics() const
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_buffer_->diagnostics();
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

  double steadyAgeSeconds(
    const std::optional<std::chrono::steady_clock::time_point> & stamp) const
  {
    if (!stamp.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - *stamp).count();
  }

  std::string rawGnssStatus() const
  {
    const double startup_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - startup_steady_).count();
    if (!last_raw_observation_reception_steady_.has_value()) {
      return startup_age >= raw_startup_grace_s_ ?
             "RAW_GNSS_UNAVAILABLE" : "WAITING_FOR_RAW_GNSS";
    }
    if (!last_valid_raw_observation_reception_steady_.has_value()) {
      return startup_age >= raw_startup_grace_s_ ?
             "RAW_GNSS_INVALID" : "WAITING_FOR_VALID_RAW_GNSS";
    }
    if (steadyAgeSeconds(last_valid_raw_observation_reception_steady_) >
      observation_stale_timeout_s_)
    {
      return "RAW_GNSS_STALE";
    }
    if (!last_ephemeris_reception_steady_.has_value()) {
      return startup_age >= raw_startup_grace_s_ ?
             "RAW_EPHEMERIS_UNAVAILABLE" : "WAITING_FOR_EPHEMERIS";
    }
    if (!last_valid_ephemeris_reception_steady_.has_value()) {
      return startup_age >= raw_startup_grace_s_ ?
             "RAW_EPHEMERIS_INVALID" : "WAITING_FOR_VALID_EPHEMERIS";
    }
    if (steadyAgeSeconds(last_valid_ephemeris_reception_steady_) >
      ephemeris_stale_timeout_s_)
    {
      return "RAW_EPHEMERIS_STALE";
    }
    return "RAW_GNSS_ACTIVE";
  }

  bool optimizeGraph()
  {
    const auto started = std::chrono::steady_clock::now();
    const bool succeeded = smoother_->optimize();
    last_optimization_latency_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    total_optimization_latency_ms_ += last_optimization_latency_ms_;
    maximum_optimization_latency_ms_ = std::max(
      maximum_optimization_latency_ms_, last_optimization_latency_ms_);
    ++measured_optimization_calls_;
    last_optimization_numerically_rejected_ = false;
    if (succeeded) {
      const double condition = smoother_->diagnostics().last_condition_estimate;
      if (!std::isfinite(condition) || condition > maximum_condition_estimate_) {
        last_optimization_numerically_rejected_ = true;
        last_rejected_condition_estimate_ = condition;
        ++numerical_condition_rejections_;
        return false;
      }
    }
    if (last_state_id_.has_value()) {
      const EcefState * latest = smoother_->state(*last_state_id_);
      if (latest != nullptr) {
        if (last_optimizer_state_stamp_s_.has_value() &&
          latest->stamp_s > *last_optimizer_state_stamp_s_)
        {
          const double input_interval_s = latest->stamp_s - *last_optimizer_state_stamp_s_;
          last_real_time_factor_ = last_optimization_latency_ms_ * 1.0e-3 / input_interval_s;
        }
        if (!last_optimizer_state_stamp_s_.has_value() ||
          latest->stamp_s > *last_optimizer_state_stamp_s_)
        {
          last_optimizer_state_stamp_s_ = latest->stamp_s;
        }
      }
    }
    return succeeded;
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
    std::lock_guard<std::mutex> lock(raw_input_mutex_);
    if (raw_ephemeris_queue_.size() >= pending_ephemeris_capacity_) {
      raw_ephemeris_queue_.pop_front();
      ++dropped_raw_ephemeris_queue_;
    }
    raw_ephemeris_queue_.push_back(message);
  }

  void processEphemeris(const gnss_raw_msgs::msg::Ephemeris::SharedPtr & message)
  {
    last_ephemeris_reception_steady_ = std::chrono::steady_clock::now();
    try {
      const BroadcastEphemeris value = ephemeris(*message);
      last_valid_ephemeris_reception_steady_ = std::chrono::steady_clock::now();
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
    std::lock_guard<std::mutex> lock(raw_input_mutex_);
    if (raw_epoch_queue_.size() >= pending_raw_epoch_capacity_) {
      raw_epoch_queue_.pop_front();
      ++dropped_raw_epoch_queue_;
    }
    raw_epoch_queue_.push_back(message);
  }

  void onReferenceStation(const gnss_raw_msgs::msg::ReferenceStation::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(raw_input_mutex_);
    if (raw_reference_station_queue_.size() >= pending_reference_station_capacity_) {
      raw_reference_station_queue_.pop_front();
      ++dropped_reference_station_queue_;
    }
    raw_reference_station_queue_.push_back(message);
  }

  bool processReferenceStation(const gnss_raw_msgs::msg::ReferenceStation & message)
  {
    ++reference_station_updates_;
    if (static_base_ecef_calibrated_ || !dynamic_base_enabled_) {
      ++ignored_reference_station_updates_;
      return false;
    }
    const ReferenceStationSample sample{
      message.station_id,
      message.message_type,
      message.itrf_realization,
      message.source,
      {message.position_ecef_m[0], message.position_ecef_m[1], message.position_ecef_m[2]}};
    const auto update = reference_station_tracker_->accept(sample);
    if (update == ReferenceStationUpdate::Rejected) {
      ++invalid_reference_station_updates_;
      return false;
    }
    dynamic_base_ecef_calibrated_ = true;
    dynamic_base_station_id_ = sample.station_id;
    dynamic_base_message_type_ = sample.message_type;
    dynamic_base_itrf_realization_ = sample.itrf_realization;
    dynamic_base_source_ = sample.source;
    base_ecef_source_ = sample.message_type == 1006U ? "RTCM_1006" : "RTCM_1005";
    if (update == ReferenceStationUpdate::Initial) {
      base_ecef_m_ = reference_station_tracker_->current()->position_ecef_m;
      return false;
    }
    if (update != ReferenceStationUpdate::Changed) {
      return false;
    }

    base_ecef_m_ = reference_station_tracker_->current()->position_ecef_m;
    epoch_aligner_ = std::make_unique<GnssEpochAligner>(epoch_aligner_config_);
    dd_builder_ = std::make_unique<DoubleDifferenceBuilder>(
      dd_builder_config_, reference_selector_config_, ambiguity_arc_config_);
    pending_gnss_.clear();
    resetGraph();
    ++reference_station_changes_;
    estimator_state_ = "REFERENCE_STATION_CHANGED";
    return true;
  }

  void processGnssEpoch(const gnss_raw_msgs::msg::ObservationEpoch::SharedPtr & message)
  {
    last_raw_observation_reception_steady_ = std::chrono::steady_clock::now();
    ++raw_observation_epochs_received_;
    if (message->receiver < raw_receiver_epochs_.size()) {
      ++raw_receiver_epochs_[message->receiver];
    }
    try {
      const GnssObservationEpoch observation = observationEpoch(*message);
      last_valid_raw_observation_reception_steady_ = std::chrono::steady_clock::now();
      ++raw_valid_observation_epochs_;
      last_raw_week_ = message->week;
      last_raw_milliseconds_of_week_ = message->milliseconds_of_week;
      last_raw_time_status_ = message->time_status;
      const auto aligned = epoch_aligner_->push(observation);
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
      }
    } catch (const std::exception &) {
      ++invalid_gnss_epochs_;
    }
  }

  void drainRawInputs()
  {
    std::deque<gnss_raw_msgs::msg::ReferenceStation::SharedPtr> reference_stations;
    std::deque<gnss_raw_msgs::msg::Ephemeris::SharedPtr> ephemerides;
    std::deque<gnss_raw_msgs::msg::ObservationEpoch::SharedPtr> epochs;
    {
      std::lock_guard<std::mutex> lock(raw_input_mutex_);
      reference_stations.swap(raw_reference_station_queue_);
      ephemerides.swap(raw_ephemeris_queue_);
      epochs.swap(raw_epoch_queue_);
    }
    bool reference_station_changed = false;
    for (const auto & message : reference_stations) {
      reference_station_changed = processReferenceStation(*message) || reference_station_changed;
    }
    if (reference_station_changed) {
      dropped_epochs_on_reference_station_change_ += epochs.size();
      epochs.clear();
    }
    for (const auto & message : ephemerides) {
      processEphemeris(message);
    }
    for (const auto & message : epochs) {
      processGnssEpoch(message);
    }
  }

  bool attachToClosestState(const AlignedGnssEpochs & epochs)
  {
    if (!baseEcefReady() || !timeSyncReady() || state_gnss_seconds_.empty()) {
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
      const auto base_observation = std::find_if(
        epochs.base.observations.begin(), epochs.base.observations.end(),
        [&observation](const GnssObservation & candidate) {
          return candidate.satellite == observation.satellite &&
          candidate.signal == observation.signal && candidate.pseudorange_valid;
        });
      if (base_observation == epochs.base.observations.end()) {
        continue;
      }
      const auto rover_propagated = satellite_propagator_->propagateToReceiveFrame(
        broadcast->second, epochs.rover.time, observation.pseudorange_m);
      const auto base_propagated = satellite_propagator_->propagateToReceiveFrame(
        broadcast->second, epochs.base.time, base_observation->pseudorange_m);
      if (rover_propagated.ok() && base_propagated.ok()) {
        satellite_states[observation.satellite] =
        {*rover_propagated.state, *base_propagated.state};
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
    integer_fix_pending_ = true;
    if (!most_recent_gnss_factor_state_.has_value() ||
      closest_id > *most_recent_gnss_factor_state_)
    {
      most_recent_gnss_factor_state_ = closest_id;
    }
    gnss_measurements_ += measurements.size();
    return true;
  }

  enum class LidarBatchResult : std::uint8_t
  {
    Consumed,
    WaitingForImu,
  };

  struct PendingLidarBatch
  {
    fgo_gil_msgs::msg::LidarConstraintBatch::SharedPtr message;
    std::chrono::steady_clock::time_point received;
    bool wait_reported = false;
  };

  void onLidarConstraints(const fgo_gil_msgs::msg::LidarConstraintBatch::SharedPtr message)
  {
    if (!message->keyframe) {
      return;
    }
    if (pending_lidar_.size() >= pending_lidar_capacity_) {
      pending_lidar_.pop_front();
      ++dropped_pending_lidar_capacity_;
    }
    pending_lidar_.push_back({message, std::chrono::steady_clock::now(), false});
    drainPendingLidar();
  }

  void drainPendingLidar()
  {
    drainRawInputs();
    while (!pending_lidar_.empty()) {
      PendingLidarBatch & pending = pending_lidar_.front();
      const double age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - pending.received).count();
      if (age_s > pending_lidar_timeout_s_) {
        pending_lidar_.pop_front();
        ++dropped_pending_lidar_timeout_;
        continue;
      }
      const LidarBatchResult result = processLidarConstraints(pending.message);
      if (result == LidarBatchResult::WaitingForImu) {
        if (!pending.wait_reported) {
          pending.wait_reported = true;
          ++lidar_batches_without_imu_;
        }
        return;
      }
      pending_lidar_.pop_front();
      return;
    }
  }

  void resetGraphForImuSegment()
  {
    resetGraph();
    pending_gnss_.clear();
    ++imu_graph_reseeds_;
    estimator_state_ = "IMU_SEGMENT_RESEEDED";
  }

  LidarBatchResult processLidarConstraints(
    const fgo_gil_msgs::msg::LidarConstraintBatch::SharedPtr & message)
  {
    if (!ecef_world_calibrated_) {
      estimator_state_ = "WAITING_FOR_CALIBRATION";
      return LidarBatchResult::Consumed;
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
      return LidarBatchResult::Consumed;
    }

    std::optional<std::vector<ImuSample>> imu_samples;
    while (true) {
      if (!last_state_id_.has_value()) {
        const auto coverage = imuCoverage(stamp_s);
        if (graph_imu_segment_id_.has_value() &&
          *graph_imu_segment_id_ != coverage.second)
        {
          resetGraphForImuSegment();
          continue;
        }
        if (coverage.first == ImuCoverageResult::WaitingForFuture) {
          estimator_state_ = "WAITING_FOR_IMU_COVERAGE";
          return LidarBatchResult::WaitingForImu;
        }
        if (coverage.first == ImuCoverageResult::HistoryUnavailable) {
          ++dropped_lidar_history_unavailable_;
          estimator_state_ = "IMU_HISTORY_UNAVAILABLE";
          return LidarBatchResult::Consumed;
        }
        if (coverage.first == ImuCoverageResult::InvalidInterval) {
          ++invalid_lidar_batches_;
          return LidarBatchResult::Consumed;
        }
        graph_imu_segment_id_ = coverage.second;
        break;
      }

      const EcefState * previous = smoother_->state(*last_state_id_);
      if (previous == nullptr) {
        resetGraphForImuSegment();
        continue;
      }
      ImuSegmentSelection selection = imuSegment(previous->stamp_s, stamp_s);
      if (graph_imu_segment_id_.has_value() &&
        *graph_imu_segment_id_ != selection.segment_id)
      {
        resetGraphForImuSegment();
        continue;
      }
      if (selection.result == ImuCoverageResult::WaitingForFuture) {
        estimator_state_ = "WAITING_FOR_IMU_COVERAGE";
        return LidarBatchResult::WaitingForImu;
      }
      if (selection.result == ImuCoverageResult::HistoryUnavailable) {
        resetGraphForImuSegment();
        continue;
      }
      if (selection.result == ImuCoverageResult::InvalidInterval) {
        ++invalid_lidar_batches_;
        return LidarBatchResult::Consumed;
      }
      graph_imu_segment_id_ = selection.segment_id;
      imu_samples = std::move(selection.samples);
      break;
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
    if (!smoother_->addState(state_id, initial)) {
      ++invalid_lidar_batches_;
      return LidarBatchResult::Consumed;
    }
    if (!last_state_id_.has_value()) {
      if (!smoother_->addStatePrior(state_id, initial, prior_noise_)) {
        resetGraph();
        return LidarBatchResult::Consumed;
      }
    } else if (!smoother_->addImuFactor(
        *last_state_id_, state_id, *imu_samples, imu_factor_config_))
    {
      resetGraph();
      estimator_state_ = "IMU_FACTOR_REJECTED";
      return LidarBatchResult::Consumed;
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
        return LidarBatchResult::Consumed;
      }
    }
    last_state_id_ = state_id;
    if (timeSyncReady()) {
      state_gnss_seconds_[state_id] = stamp_s + *ros_to_gnss_offset_s_;
      attachPendingGnss(state_id);
    } else {
      smoother_->recordGnssOutage();
    }
    if (!optimizeGraph()) {
      ++optimization_failures_;
      const bool numerically_rejected = last_optimization_numerically_rejected_;
      resetGraph();
      pending_gnss_.clear();
      estimator_state_ = numerically_rejected ?
        "NUMERICAL_CONDITION_REJECTED_RESET" : "OPTIMIZATION_FAILED_RESET";
      return LidarBatchResult::Consumed;
    }
    pruneStateBookkeeping();
    if (baseEcefReady()) {
      updateIntegerSolution();
      estimator_state_ = solution_status_ == "FIXED" ? "FIXED_ACTIVE" : "FLOAT_ACTIVE";
    } else {
      fixed_state_.reset();
      solution_status_ = "FLOAT";
      estimator_state_ = "LIO_ONLY_WAITING_BASE";
    }
    ++lidar_keyframes_;
    publishOdometry();
    return LidarBatchResult::Consumed;
  }

  void updateIntegerSolution()
  {
    fixed_state_.reset();
    solution_status_ = "FLOAT";
    if (!last_state_id_.has_value() || !most_recent_gnss_factor_state_.has_value() ||
      !integer_fix_pending_ || smoother_->state(*most_recent_gnss_factor_state_) == nullptr)
    {
      return;
    }
    integer_fix_pending_ = false;
    ++integer_fix_attempts_;
    last_back_substitution_ = {};
    const auto estimate = smoother_->floatAmbiguityEstimate();
    if (!estimate.has_value()) {
      integer_confirmation_->reset();
      last_integer_fix_ = {};
      last_integer_fix_.rejection_reason =
        IntegerFixRejectionReason::CovarianceNotPositiveDefinite;
      ++integer_fix_rejections_;
      return;
    }
    last_integer_fix_ = integer_resolver_->resolve(*estimate);
    if (!last_integer_fix_.fixed) {
      integer_confirmation_->reset();
      ++integer_fix_rejections_;
      return;
    }
    last_back_substitution_ = smoother_->previewFixedAmbiguities(
      last_integer_fix_.keys, last_integer_fix_.fixed_values_m,
      fixed_back_substitution_config_);
    if (!last_back_substitution_.accepted) {
      integer_confirmation_->reset();
      last_integer_fix_.fixed = false;
      last_integer_fix_.rejection_reason =
        IntegerFixRejectionReason::BackSubstitutionRejected;
      ++integer_fix_rejections_;
      return;
    }
    if (!integer_confirmation_->update(last_integer_fix_)) {
      last_integer_fix_.fixed = false;
      last_integer_fix_.rejection_reason = IntegerFixRejectionReason::ConfirmationPending;
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

  nav_msgs::msg::Odometry odometryMessage(const EcefState & state) const
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(
      static_cast<std::int64_t>(std::llround(state.stamp_s * 1.0e9)));
    message.header.frame_id = ecef_frame_;
    message.child_frame_id = body_frame_;
    message.pose.pose.position.x = state.position_ecef_m.x;
    message.pose.pose.position.y = state.position_ecef_m.y;
    message.pose.pose.position.z = state.position_ecef_m.z;
    message.pose.pose.orientation.w = state.orientation_ecef_body.w;
    message.pose.pose.orientation.x = state.orientation_ecef_body.x;
    message.pose.pose.orientation.y = state.orientation_ecef_body.y;
    message.pose.pose.orientation.z = state.orientation_ecef_body.z;
    message.twist.twist.linear.x = state.velocity_ecef_m_s.x;
    message.twist.twist.linear.y = state.velocity_ecef_m_s.y;
    message.twist.twist.linear.z = state.velocity_ecef_m_s.z;
    return message;
  }

  bool validOutputState(const EcefState & state) const
  {
    return std::isfinite(state.stamp_s) && finite(state.position_ecef_m) &&
           finite(state.orientation_ecef_body) && finite(state.velocity_ecef_m_s) &&
           norm(state.position_ecef_m) > 1.0e6;
  }

  void publishOdometry()
  {
    if (!last_state_id_.has_value()) {
      return;
    }
    const EcefState * float_state = smoother_->state(*last_state_id_);
    if (float_state == nullptr || !validOutputState(*float_state) ||
      (fixed_state_.has_value() && !validOutputState(*fixed_state_)))
    {
      ++nonfinite_output_rejections_;
      return;
    }

    odometry_pub_->publish(odometryMessage(*float_state));
    if (fixed_state_.has_value()) {
      fixed_odometry_pub_->publish(odometryMessage(*fixed_state_));
    }
    const EcefState & selected_state = fixed_state_.has_value() ? *fixed_state_ : *float_state;
    nav_msgs::msg::Odometry selected_message = odometryMessage(selected_state);
    output_odometry_pub_->publish(selected_message);

    geometry_msgs::msg::PoseStamped pose_message;
    pose_message.header = selected_message.header;
    pose_message.pose = selected_message.pose.pose;
    if (!path_msg_.poses.empty() &&
      path_msg_.poses.back().header.stamp == pose_message.header.stamp)
    {
      path_msg_.poses.back() = pose_message;
    } else {
      path_msg_.poses.push_back(pose_message);
      if (path_msg_.poses.size() > maximum_path_poses_) {
        path_msg_.poses.erase(path_msg_.poses.begin());
      }
    }
    path_msg_.header = selected_message.header;
    path_pub_->publish(path_msg_);
    last_output_stamp_s_ = selected_state.stamp_s;
    last_output_reception_steady_ = std::chrono::steady_clock::now();
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
    drainRawInputs();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fgo_gil/float_fgo";
    status.hardware_id = "jetson_orin_nx";
    const std::string raw_status = rawGnssStatus();
    status.message = estimator_state_;
    if (raw_status == "RAW_GNSS_UNAVAILABLE" || raw_status == "RAW_GNSS_INVALID" ||
      raw_status == "RAW_GNSS_STALE" || raw_status == "RAW_EPHEMERIS_UNAVAILABLE" ||
      raw_status == "RAW_EPHEMERIS_INVALID" || raw_status == "RAW_EPHEMERIS_STALE")
    {
      status.message = raw_status;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (estimator_state_ == "FLOAT_ACTIVE" || estimator_state_ == "FIXED_ACTIVE") {
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
    const double runtime_s = std::max(
      1.0e-9, std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startup_steady_).count());
    const double output_age_s = steadyAgeSeconds(last_output_reception_steady_);
    const double output_stamp_age_s = last_output_stamp_s_.has_value() ?
      now().seconds() - *last_output_stamp_s_ : std::numeric_limits<double>::infinity();
    const double residual_rms = graph.last_residual_rows == 0U ? 0.0 :
      std::sqrt(2.0 * graph.last_cost / static_cast<double>(graph.last_residual_rows));
    status.values.push_back(keyValue("time_sync_state", time_sync_state_));
    status.values.push_back(keyValue("raw_gnss_status", raw_status));
    status.values.push_back(
      numericKeyValue(
        "raw_observation_age_s", steadyAgeSeconds(last_raw_observation_reception_steady_)));
    status.values.push_back(
      numericKeyValue(
        "valid_raw_observation_age_s", steadyAgeSeconds(
          last_valid_raw_observation_reception_steady_)));
    status.values.push_back(
      numericKeyValue("ephemeris_age_s", steadyAgeSeconds(last_ephemeris_reception_steady_)));
    status.values.push_back(
      numericKeyValue(
        "valid_ephemeris_age_s", steadyAgeSeconds(
          last_valid_ephemeris_reception_steady_)));
    status.values.push_back(
      numericKeyValue("raw_observation_rate_hz", raw_observation_epochs_received_ / runtime_s));
    status.values.push_back(
      numericKeyValue(
        "valid_raw_observation_rate_hz", raw_valid_observation_epochs_ / runtime_s));
    status.values.push_back(numericKeyValue("raw_week", last_raw_week_));
    status.values.push_back(
      numericKeyValue("raw_milliseconds_of_week", last_raw_milliseconds_of_week_));
    status.values.push_back(numericKeyValue("raw_time_status", last_raw_time_status_));
    status.values.push_back(numericKeyValue("raw_master_epochs", raw_receiver_epochs_[1]));
    status.values.push_back(numericKeyValue("raw_secondary_epochs", raw_receiver_epochs_[2]));
    status.values.push_back(numericKeyValue("raw_base_epochs", raw_receiver_epochs_[3]));
    status.values.push_back(
      keyValue("ecef_world_calibrated", ecef_world_calibrated_ ? "true" : "false"));
    status.values.push_back(
      keyValue("base_ecef_calibrated", baseEcefReady() ? "true" : "false"));
    status.values.push_back(
      keyValue("base_ecef_source", base_ecef_source_));
    status.values.push_back(
      numericKeyValue("dynamic_base_station_id", dynamic_base_station_id_));
    status.values.push_back(
      numericKeyValue("dynamic_base_message_type", dynamic_base_message_type_));
    status.values.push_back(
      numericKeyValue("dynamic_base_itrf_realization", dynamic_base_itrf_realization_));
    status.values.push_back(
      keyValue("dynamic_base_source", dynamic_base_source_));
    status.values.push_back(
      numericKeyValue("reference_station_updates", reference_station_updates_));
    status.values.push_back(
      numericKeyValue("reference_station_changes", reference_station_changes_));
    status.values.push_back(
      numericKeyValue("invalid_reference_station_updates", invalid_reference_station_updates_));
    status.values.push_back(
      numericKeyValue("ignored_reference_station_updates", ignored_reference_station_updates_));
    status.values.push_back(
      numericKeyValue("dropped_reference_station_queue", dropped_reference_station_queue_));
    status.values.push_back(
      numericKeyValue(
        "dropped_epochs_on_reference_station_change",
        dropped_epochs_on_reference_station_change_));
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
    status.values.push_back(
      numericKeyValue(
        "fixed_ambiguities", last_integer_fix_.fixed ? last_integer_fix_.keys.size() : 0U));
    status.values.push_back(
      numericKeyValue(
        "candidate_ambiguities", last_integer_fix_.keys.size()));
    status.values.push_back(
      numericKeyValue("eligible_ambiguities", last_integer_fix_.eligible_ambiguities));
    status.values.push_back(
      numericKeyValue("evaluated_ambiguities", last_integer_fix_.evaluated_ambiguities));
    status.values.push_back(
      numericKeyValue("fractional_cycle_rms", last_integer_fix_.fractional_cycle_rms));
    status.values.push_back(
      numericKeyValue("fractional_cycle_max", last_integer_fix_.fractional_cycle_max));
    status.values.push_back(
      numericKeyValue("integer_fixed_solutions", integer_fixed_solutions_));
    status.values.push_back(numericKeyValue("integer_fix_attempts", integer_fix_attempts_));
    status.values.push_back(
      numericKeyValue("integer_confirmation_count", integer_confirmation_->count()));
    status.values.push_back(
      numericKeyValue("integer_fix_rejections", integer_fix_rejections_));
    status.values.push_back(
      numericKeyValue(
        "fixed_position_correction_m", last_back_substitution_.maximum_position_correction_m));
    status.values.push_back(
      numericKeyValue(
        "fixed_rotation_correction_rad", last_back_substitution_.maximum_rotation_correction_rad));
    status.values.push_back(
      numericKeyValue(
        "fixed_cost_increase", last_back_substitution_.cost_after -
        last_back_substitution_.cost_before));
    status.values.push_back(numericKeyValue("clock_uncertainty_s", clock_uncertainty_s_));
    status.values.push_back(numericKeyValue("states", graph.states));
    status.values.push_back(numericKeyValue("ambiguities", graph.ambiguities));
    status.values.push_back(numericKeyValue("factors", graph.factors));
    status.values.push_back(numericKeyValue("window_span_s", graph.window_span_s));
    status.values.push_back(
      numericKeyValue("optimization_rollbacks", graph.optimization_rollbacks));
    status.values.push_back(numericKeyValue("marginalizations", graph.marginalizations));
    status.values.push_back(numericKeyValue("gnss_outages", graph.gnss_outages));
    status.values.push_back(numericKeyValue("last_iterations", graph.last_iterations));
    status.values.push_back(numericKeyValue("last_delta_norm", graph.last_delta_norm));
    status.values.push_back(numericKeyValue("last_cost", graph.last_cost));
    status.values.push_back(numericKeyValue("last_residual_rms", residual_rms));
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
    status.values.push_back(
      numericKeyValue("numerical_condition_rejections", numerical_condition_rejections_));
    status.values.push_back(
      numericKeyValue("last_rejected_condition_estimate", last_rejected_condition_estimate_));
    status.values.push_back(numericKeyValue("graph_resets", graph_resets_));
    status.values.push_back(numericKeyValue("imu_graph_reseeds", imu_graph_reseeds_));
    status.values.push_back(
      numericKeyValue("lidar_batches_waiting_for_imu", lidar_batches_without_imu_));
    status.values.push_back(
      numericKeyValue("pending_lidar_batches", pending_lidar_.size()));
    status.values.push_back(
      numericKeyValue("dropped_pending_lidar_timeout", dropped_pending_lidar_timeout_));
    status.values.push_back(
      numericKeyValue("dropped_pending_lidar_capacity", dropped_pending_lidar_capacity_));
    status.values.push_back(
      numericKeyValue("dropped_lidar_history_unavailable", dropped_lidar_history_unavailable_));
    {
      std::lock_guard<std::mutex> lock(raw_input_mutex_);
      status.values.push_back(
        numericKeyValue("pending_raw_epochs", raw_epoch_queue_.size()));
      status.values.push_back(
        numericKeyValue("pending_raw_ephemerides", raw_ephemeris_queue_.size()));
      status.values.push_back(
        numericKeyValue("dropped_raw_epoch_queue", dropped_raw_epoch_queue_));
      status.values.push_back(
        numericKeyValue("dropped_raw_ephemeris_queue", dropped_raw_ephemeris_queue_));
    }
    status.values.push_back(numericKeyValue("output_age_s", output_age_s));
    status.values.push_back(numericKeyValue("output_stamp_age_s", output_stamp_age_s));
    status.values.push_back(
      keyValue("shadow_only", (!publish_tf_ && !nav2_use_fgo_) ? "true" : "false"));
    const ImuBufferDiagnostics imu = imuDiagnostics();
    status.values.push_back(numericKeyValue("imu_segment_id", imu.segment_id));
    status.values.push_back(numericKeyValue("imu_gaps", imu.gaps));
    status.values.push_back(numericKeyValue("imu_time_reversals", imu.time_reversals));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));

    diagnostic_msgs::msg::DiagnosticArray factor_array;
    factor_array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus factor_status;
    factor_status.name = "fgo_gil/factors";
    factor_status.hardware_id = "jetson_orin_nx";
    factor_status.level = graph.last_solve_succeeded ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    factor_status.message = graph.last_solve_succeeded ? "GRAPH_OK" : "GRAPH_NOT_SOLVED";
    factor_status.values.push_back(
      numericKeyValue(
        "state_prior_factors",
        graph.state_prior_factors));
    factor_status.values.push_back(numericKeyValue("imu_factors", graph.imu_factors));
    factor_status.values.push_back(
      numericKeyValue("lidar_line_factors", graph.lidar_line_factors));
    factor_status.values.push_back(
      numericKeyValue("lidar_plane_factors", graph.lidar_plane_factors));
    factor_status.values.push_back(numericKeyValue("gnss_code_factors", graph.gnss_code_factors));
    factor_status.values.push_back(
      numericKeyValue("gnss_carrier_factors", graph.gnss_carrier_factors));
    factor_status.values.push_back(numericKeyValue("residual_rows", graph.last_residual_rows));
    factor_status.values.push_back(numericKeyValue("residual_rms", residual_rms));
    factor_status.values.push_back(numericKeyValue("rejected_factors", graph.rejected_factors));
    factor_status.values.push_back(numericKeyValue("dd_code_factors_total", dd.code_factors));
    factor_status.values.push_back(
      numericKeyValue("dd_carrier_factors_total", dd.carrier_factors));
    factor_status.values.push_back(numericKeyValue("dd_new_arcs_total", dd.new_arcs));
    for (std::size_t index = 1; index < dd.arc_resets.size(); ++index) {
      const auto reason = static_cast<ArcResetReason>(index);
      factor_status.values.push_back(
        numericKeyValue(std::string("arc_reset_") + toString(reason), dd.arc_resets[index]));
    }
    for (std::size_t index = 0; index < dd.rejected.size(); ++index) {
      const auto reason = static_cast<DdRejectReason>(index);
      factor_status.values.push_back(
        numericKeyValue(std::string("dd_rejected_") + toString(reason), dd.rejected[index]));
    }
    factor_array.status.push_back(std::move(factor_status));
    for (const CodeResidualDiagnostics & code : smoother_->codeResidualDiagnostics()) {
      diagnostic_msgs::msg::DiagnosticStatus code_status;
      code_status.name = "fgo_gil/code_residual/" + signalGroupLabel(code.group);
      code_status.hardware_id = "um982_raw";
      code_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      code_status.message = "CODE_RESIDUAL_OBSERVED";
      code_status.values.push_back(numericKeyValue("factors", code.factors));
      code_status.values.push_back(numericKeyValue("raw_rms_m", code.raw_rms_m));
      code_status.values.push_back(numericKeyValue("raw_max_m", code.raw_max_m));
      code_status.values.push_back(numericKeyValue("normalized_rms", code.normalized_rms));
      code_status.values.push_back(numericKeyValue("normalized_max", code.normalized_max));
      code_status.values.push_back(numericKeyValue("sigma_mean_m", code.sigma_mean_m));
      code_status.values.push_back(numericKeyValue("sigma_min_m", code.sigma_min_m));
      code_status.values.push_back(numericKeyValue("sigma_max_m", code.sigma_max_m));
      factor_array.status.push_back(std::move(code_status));
    }
    for (const CarrierResidualDiagnostics & carrier :
      smoother_->carrierResidualDiagnostics(integer_resolver_config_.minimum_observation_epochs))
    {
      diagnostic_msgs::msg::DiagnosticStatus carrier_status;
      carrier_status.name = "fgo_gil/carrier_residual/" + signalGroupLabel(carrier.group);
      carrier_status.hardware_id = "um982_raw";
      carrier_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      carrier_status.message = "CARRIER_RESIDUAL_OBSERVED";
      std::size_t candidate_ambiguities = 0U;
      for (const DdAmbiguityKey & key : last_integer_fix_.keys) {
        candidate_ambiguities += static_cast<std::size_t>(key.group == carrier.group);
      }
      carrier_status.values.push_back(numericKeyValue("factors", carrier.factors));
      carrier_status.values.push_back(numericKeyValue("raw_rms_m", carrier.raw_rms_m));
      carrier_status.values.push_back(numericKeyValue("raw_max_m", carrier.raw_max_m));
      carrier_status.values.push_back(
        numericKeyValue("normalized_rms", carrier.normalized_rms));
      carrier_status.values.push_back(
        numericKeyValue("normalized_max", carrier.normalized_max));
      carrier_status.values.push_back(numericKeyValue("sigma_mean_m", carrier.sigma_mean_m));
      carrier_status.values.push_back(numericKeyValue("sigma_min_m", carrier.sigma_min_m));
      carrier_status.values.push_back(numericKeyValue("sigma_max_m", carrier.sigma_max_m));
      carrier_status.values.push_back(
        numericKeyValue("minimum_arc_observations", carrier.minimum_arc_observations));
      carrier_status.values.push_back(
        numericKeyValue("maximum_arc_observations", carrier.maximum_arc_observations));
      carrier_status.values.push_back(
        numericKeyValue("fix_eligible_ambiguities", carrier.fix_eligible_ambiguities));
      carrier_status.values.push_back(
        numericKeyValue("candidate_ambiguities", candidate_ambiguities));
      carrier_status.values.push_back(
        numericKeyValue("confirmation_count", integer_confirmation_->count()));
      factor_array.status.push_back(std::move(carrier_status));
    }
    factor_diagnostics_pub_->publish(std::move(factor_array));

    diagnostic_msgs::msg::DiagnosticArray ambiguity_array;
    ambiguity_array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus ambiguity_status;
    ambiguity_status.name = "fgo_gil/ambiguity";
    ambiguity_status.hardware_id = "um982_raw";
    ambiguity_status.level = solution_status_ == "FIXED" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    ambiguity_status.message = solution_status_ == "FIXED" ?
      "FIXED" : toString(last_integer_fix_.rejection_reason);
    ambiguity_status.values.push_back(keyValue("solution_status", solution_status_));
    ambiguity_status.values.push_back(numericKeyValue("ambiguity_dimension", graph.ambiguities));
    ambiguity_status.values.push_back(
      numericKeyValue("eligible_dimension", last_integer_fix_.eligible_ambiguities));
    ambiguity_status.values.push_back(
      numericKeyValue("evaluated_dimension", last_integer_fix_.evaluated_ambiguities));
    ambiguity_status.values.push_back(
      numericKeyValue("fractional_cycle_rms", last_integer_fix_.fractional_cycle_rms));
    ambiguity_status.values.push_back(
      numericKeyValue("fractional_cycle_max", last_integer_fix_.fractional_cycle_max));
    ambiguity_status.values.push_back(numericKeyValue("ratio", last_integer_fix_.ratio));
    ambiguity_status.values.push_back(
      numericKeyValue("success_rate", last_integer_fix_.success_rate));
    ambiguity_status.values.push_back(
      numericKeyValue(
        "fixed_ambiguities", last_integer_fix_.fixed ? last_integer_fix_.keys.size() : 0U));
    ambiguity_status.values.push_back(
      keyValue("rejection_reason", toString(last_integer_fix_.rejection_reason)));
    ambiguity_status.values.push_back(
      keyValue("back_substitution", toString(last_back_substitution_.rejection)));
    ambiguity_status.values.push_back(
      numericKeyValue("fix_attempts", integer_fix_attempts_));
    ambiguity_status.values.push_back(
      numericKeyValue("confirmation_count", integer_confirmation_->count()));
    ambiguity_status.values.push_back(
      numericKeyValue("fixed_solutions", integer_fixed_solutions_));
    ambiguity_status.values.push_back(
      keyValue("fix_pending", integer_fix_pending_ ? "true" : "false"));
    ambiguity_status.values.push_back(
      numericKeyValue("fix_rejections", integer_fix_rejections_));
    ambiguity_status.values.push_back(
      numericKeyValue("optimization_rollbacks", graph.optimization_rollbacks));
    ambiguity_array.status.push_back(std::move(ambiguity_status));
    ambiguity_status_pub_->publish(std::move(ambiguity_array));

    diagnostic_msgs::msg::DiagnosticArray performance_array;
    performance_array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus performance_status;
    performance_status.name = "fgo_gil/performance";
    performance_status.hardware_id = "jetson_orin_nx";
    const bool output_stale = output_age_s > observation_stale_timeout_s_;
    performance_status.level =
      (nonfinite_output_rejections_ != 0U || last_optimization_numerically_rejected_) ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (last_real_time_factor_ > 1.0 || output_stale ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK);
    performance_status.message = nonfinite_output_rejections_ != 0U ?
      "NONFINITE_OUTPUT_REJECTED" :
      (last_optimization_numerically_rejected_ ?
      "NUMERICAL_CONDITION_REJECTED" : (output_stale ? "OUTPUT_STALE" : "SHADOW_ONLY"));
    performance_status.values.push_back(
      numericKeyValue("optimization_latency_ms", last_optimization_latency_ms_));
    performance_status.values.push_back(
      numericKeyValue(
        "optimization_latency_mean_ms", measured_optimization_calls_ == 0U ? 0.0 :
        total_optimization_latency_ms_ / static_cast<double>(measured_optimization_calls_)));
    performance_status.values.push_back(
      numericKeyValue("optimization_latency_max_ms", maximum_optimization_latency_ms_));
    performance_status.values.push_back(
      numericKeyValue("real_time_factor", last_real_time_factor_));
    performance_status.values.push_back(numericKeyValue("window_span_s", graph.window_span_s));
    performance_status.values.push_back(numericKeyValue("states", graph.states));
    performance_status.values.push_back(numericKeyValue("factors", graph.factors));
    performance_status.values.push_back(numericKeyValue("output_age_s", output_age_s));
    performance_status.values.push_back(numericKeyValue("output_stamp_age_s", output_stamp_age_s));
    performance_status.values.push_back(
      keyValue("output_stale", output_stale ? "true" : "false"));
    performance_status.values.push_back(
      numericKeyValue("nonfinite_output_rejections", nonfinite_output_rejections_));
    performance_status.values.push_back(keyValue("publish_tf", "false"));
    performance_status.values.push_back(keyValue("nav2_use_fgo", "false"));
    performance_status.values.push_back(keyValue("control_owner", "NONE_SHADOW"));
    performance_array.status.push_back(std::move(performance_status));
    performance_pub_->publish(std::move(performance_array));
  }

  std::string imu_topic_;
  std::string lidar_constraints_topic_;
  std::string gnss_epoch_topic_;
  std::string ephemeris_topic_;
  std::string reference_station_topic_;
  std::string time_sync_topic_;
  std::string odometry_topic_;
  std::string fixed_odometry_topic_;
  std::string output_odometry_topic_;
  std::string path_topic_;
  std::string diagnostics_topic_;
  std::string factor_diagnostics_topic_;
  std::string ambiguity_status_topic_;
  std::string performance_topic_;
  std::string ecef_frame_;
  std::string body_frame_;
  bool ecef_world_calibrated_ = false;
  bool static_base_ecef_calibrated_ = false;
  bool dynamic_base_enabled_ = true;
  bool dynamic_base_ecef_calibrated_ = false;
  std::string base_ecef_source_ = "WAITING";
  std::uint16_t dynamic_base_station_id_ = 0U;
  std::uint16_t dynamic_base_message_type_ = 0U;
  std::uint8_t dynamic_base_itrf_realization_ = 0U;
  std::string dynamic_base_source_;
  RigidPose ecef_world_;
  RigidPose imu_lidar_;
  Vec3 base_ecef_m_;
  Vec3 master_in_imu_m_;
  std::size_t pending_gnss_capacity_ = 128;
  std::size_t imu_qos_depth_ = 512;
  std::size_t raw_input_qos_depth_ = 512;
  std::size_t pending_lidar_capacity_ = 16;
  std::size_t pending_raw_epoch_capacity_ = 1024;
  std::size_t pending_ephemeris_capacity_ = 64;
  std::size_t pending_reference_station_capacity_ = 16;
  std::size_t maximum_ephemerides_ = 256;
  std::size_t maximum_path_poses_ = 2000;
  std::size_t minimum_consecutive_fixes_ = 3;
  double acceleration_scale_ = 9.80665;
  double maximum_gnss_keyframe_offset_s_ = 0.10;
  double maximum_sync_age_s_ = 2.0;
  double maximum_clock_uncertainty_s_ = 0.05;
  double diagnostics_period_s_ = 1.0;
  double raw_startup_grace_s_ = 5.0;
  double observation_stale_timeout_s_ = 2.0;
  double ephemeris_stale_timeout_s_ = 300.0;
  double pending_lidar_timeout_s_ = 0.5;
  double maximum_condition_estimate_ = 1.0e12;
  bool publish_tf_ = false;
  bool nav2_use_fgo_ = false;
  FloatSmootherConfig smoother_config_;
  LidarGraphFactorConfig lidar_factor_config_;
  GnssGraphFactorConfig gnss_factor_config_;
  IntegerAmbiguityResolverConfig integer_resolver_config_;
  FixedBackSubstitutionConfig fixed_back_substitution_config_;
  StateFactorNoise prior_noise_;
  ImuGraphFactorConfig imu_factor_config_;
  EpochAlignerConfig epoch_aligner_config_;
  DoubleDifferenceBuilderConfig dd_builder_config_;
  ReferenceSelectorConfig reference_selector_config_;
  AmbiguityArcConfig ambiguity_arc_config_;

  std::unique_ptr<ImuSegmentBuffer> imu_buffer_;
  std::unique_ptr<GnssEpochAligner> epoch_aligner_;
  std::unique_ptr<DoubleDifferenceBuilder> dd_builder_;
  std::unique_ptr<ReferenceStationTracker> reference_station_tracker_;
  std::unique_ptr<SatellitePropagator> satellite_propagator_;
  std::unique_ptr<IntegerAmbiguityResolver> integer_resolver_;
  std::unique_ptr<IntegerCandidateConfirmation> integer_confirmation_;
  std::unique_ptr<FloatFixedLagSmoother> smoother_;
  mutable std::mutex imu_mutex_;
  mutable std::mutex raw_input_mutex_;
  std::map<SatelliteId, BroadcastEphemeris> ephemerides_;
  std::deque<SatelliteId> ephemeris_order_;
  std::deque<AlignedGnssEpochs> pending_gnss_;
  std::deque<PendingLidarBatch> pending_lidar_;
  std::deque<gnss_raw_msgs::msg::ObservationEpoch::SharedPtr> raw_epoch_queue_;
  std::deque<gnss_raw_msgs::msg::Ephemeris::SharedPtr> raw_ephemeris_queue_;
  std::deque<gnss_raw_msgs::msg::ReferenceStation::SharedPtr> raw_reference_station_queue_;
  std::map<StateId, double> state_gnss_seconds_;
  std::set<StateId> gnss_factor_states_;
  std::optional<StateId> last_state_id_;
  std::optional<StateId> most_recent_gnss_factor_state_;
  bool integer_fix_pending_ = false;
  std::optional<EcefState> fixed_state_;
  std::optional<std::uint64_t> active_frontend_epoch_;
  std::optional<std::uint64_t> graph_imu_segment_id_;
  StateId next_state_id_ = 1;
  std::string estimator_state_ = "WAITING_FOR_INPUT";
  std::string solution_status_ = "FLOAT";
  IntegerFixResult last_integer_fix_;
  FixedBackSubstitutionResult last_back_substitution_;
  std::string time_sync_state_ = "UNSYNCED";
  std::optional<double> ros_to_gnss_offset_s_;
  std::optional<double> last_time_sync_reception_s_;
  double clock_uncertainty_s_ = std::numeric_limits<double>::infinity();
  nav_msgs::msg::Path path_msg_;
  const std::chrono::steady_clock::time_point startup_steady_ =
    std::chrono::steady_clock::now();
  std::optional<std::chrono::steady_clock::time_point>
  last_raw_observation_reception_steady_;
  std::optional<std::chrono::steady_clock::time_point>
  last_valid_raw_observation_reception_steady_;
  std::optional<std::chrono::steady_clock::time_point> last_ephemeris_reception_steady_;
  std::optional<std::chrono::steady_clock::time_point> last_valid_ephemeris_reception_steady_;
  std::optional<std::chrono::steady_clock::time_point> last_output_reception_steady_;
  std::optional<double> last_output_stamp_s_;
  std::optional<double> last_optimizer_state_stamp_s_;
  std::array<std::uint64_t, 4> raw_receiver_epochs_{};
  std::uint32_t last_raw_week_ = 0;
  std::uint32_t last_raw_milliseconds_of_week_ = 0;
  std::uint8_t last_raw_time_status_ = 0;
  double last_optimization_latency_ms_ = 0.0;
  double total_optimization_latency_ms_ = 0.0;
  double maximum_optimization_latency_ms_ = 0.0;
  double last_real_time_factor_ = 0.0;
  double last_rejected_condition_estimate_ = 0.0;
  bool last_optimization_numerically_rejected_ = false;

  std::uint64_t graph_resets_ = 0;
  std::uint64_t lidar_keyframes_ = 0;
  std::uint64_t lidar_batches_without_imu_ = 0;
  std::uint64_t imu_graph_reseeds_ = 0;
  std::uint64_t dropped_pending_lidar_timeout_ = 0;
  std::uint64_t dropped_pending_lidar_capacity_ = 0;
  std::uint64_t dropped_lidar_history_unavailable_ = 0;
  std::uint64_t dropped_raw_epoch_queue_ = 0;
  std::uint64_t dropped_raw_ephemeris_queue_ = 0;
  std::uint64_t dropped_reference_station_queue_ = 0;
  std::uint64_t dropped_epochs_on_reference_station_change_ = 0;
  std::uint64_t reference_station_updates_ = 0;
  std::uint64_t reference_station_changes_ = 0;
  std::uint64_t invalid_reference_station_updates_ = 0;
  std::uint64_t ignored_reference_station_updates_ = 0;
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
  std::uint64_t numerical_condition_rejections_ = 0;
  std::uint64_t integer_fix_attempts_ = 0;
  std::uint64_t integer_fixed_solutions_ = 0;
  std::uint64_t integer_fix_rejections_ = 0;
  std::uint64_t raw_observation_epochs_received_ = 0;
  std::uint64_t raw_valid_observation_epochs_ = 0;
  std::uint64_t measured_optimization_calls_ = 0;
  std::uint64_t nonfinite_output_rejections_ = 0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fixed_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr factor_diagnostics_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr ambiguity_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr performance_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<fgo_gil_msgs::msg::LidarConstraintBatch>::SharedPtr lidar_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr gnss_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::Ephemeris>::SharedPtr ephemeris_sub_;
  rclcpp::Subscription<gnss_raw_msgs::msg::ReferenceStation>::SharedPtr reference_station_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr time_sync_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr pending_lidar_timer_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr estimator_callback_group_;
  rclcpp::CallbackGroup::SharedPtr raw_input_callback_group_;
};

}  // namespace fgo_gil_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<fgo_gil_localizer::FloatFgoNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fgo_gil_float_fgo"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
