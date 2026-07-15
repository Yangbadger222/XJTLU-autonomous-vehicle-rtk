#include "um982_rtk_driver/nmea_parser.hpp"
#include "um982_rtk_driver/ntrip_response.hpp"
#include "um982_raw_driver/ephemeris_decoder.hpp"
#include "um982_raw_driver/epoch_deduplicator.hpp"
#include "um982_raw_driver/mixed_stream_framer.hpp"
#include "um982_raw_driver/observation_decoder.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <gnss_raw_msgs/msg/ephemeris.hpp>
#include <gnss_raw_msgs/msg/observation.hpp>
#include <gnss_raw_msgs/msg/observation_epoch.hpp>
#include <gnss_raw_msgs/msg/raw_frame.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace um982_rtk_driver
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string base64Encode(const std::string & input)
{
  static constexpr char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  int value = 0;
  int value_bits = -6;
  for (const unsigned char c : input) {
    value = (value << 8) + c;
    value_bits += 8;
    while (value_bits >= 0) {
      output.push_back(alphabet[(value >> value_bits) & 0x3F]);
      value_bits -= 6;
    }
  }
  if (value_bits > -6) {
    output.push_back(alphabet[((value << 8) >> (value_bits + 8)) & 0x3F]);
  }
  while (output.size() % 4 != 0) {
    output.push_back('=');
  }
  return output;
}

std::string readPasswordFromEnv(const std::string & parameter_value, const std::string & env_name)
{
  if (!parameter_value.empty()) {
    return parameter_value;
  }
  if (env_name.empty()) {
    return "";
  }
  const char * value = std::getenv(env_name.c_str());
  if (value == nullptr) {
    return "";
  }
  return std::string(value);
}

builtin_interfaces::msg::Time toRosTimeMsg(const rclcpp::Time & stamp)
{
  builtin_interfaces::msg::Time msg;
  const int64_t nanoseconds = stamp.nanoseconds();
  msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return msg;
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

}  // namespace

class Um982RtkNode : public rclcpp::Node
{
public:
  Um982RtkNode()
  : Node("um982_rtk_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/rtk_um982");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "gps");
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    status_period_s_ = declare_parameter<double>("status_period_s", 1.0);
    heading_offset_deg_ = declare_parameter<double>("heading_offset_deg", 0.0);
    epe_quality_0_ = declare_parameter<double>("epe_quality0", 1000000.0);
    epe_quality_1_ = declare_parameter<double>("epe_quality1", 4.0);
    epe_quality_2_ = declare_parameter<double>("epe_quality2", 0.1);
    epe_quality_4_ = declare_parameter<double>("epe_quality4", 0.02);
    epe_quality_5_ = declare_parameter<double>("epe_quality5", 4.0);
    epe_quality_9_ = declare_parameter<double>("epe_quality9", 3.0);

    stream_mode_ = declare_parameter<std::string>("stream_mode", "nmea_only");
    read_chunk_bytes_ = declare_parameter<int>("read_chunk_bytes", 4096);
    max_read_batches_ = declare_parameter<int>("max_read_batches_per_cycle", 16);
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 20);
    max_payload_bytes_ = declare_parameter<int>("max_payload_bytes", 65535);
    max_buffer_bytes_ = declare_parameter<int>("max_buffer_bytes", 131072);
    max_ascii_line_bytes_ = declare_parameter<int>("max_ascii_line_bytes", 1024);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 1.0);
    raw_frame_id_ = declare_parameter<std::string>("raw.frame_id", "gnss_raw");
    raw_output_topic_ =
      declare_parameter<std::string>("raw.output_topic", "/gnss/raw/frame");
    observation_topic_ = declare_parameter<std::string>(
      "raw.observation_topic", "/gnss/raw/observation_epoch");
    ephemeris_topic_ =
      declare_parameter<std::string>("raw.ephemeris_topic", "/gnss/raw/ephemeris");
    diagnostics_topic_ =
      declare_parameter<std::string>("raw.diagnostics_topic", "/gnss/raw/diagnostics");
    epoch_dedup_capacity_ = declare_parameter<int>("raw.epoch_dedup_capacity", 256);
    stale_ascii_timeout_s_ = declare_parameter<double>("stale_ascii_timeout_s", 2.0);
    stale_binary_timeout_s_ = declare_parameter<double>("raw.stale_frame_timeout_s", 2.0);

    validateStreamParameters();
    mixed_mode_ = stream_mode_ == "mixed";
    mixed_framer_ = std::make_unique<um982_raw_driver::MixedStreamFramer>(
      static_cast<std::size_t>(max_payload_bytes_),
      static_cast<std::size_t>(max_buffer_bytes_),
      static_cast<std::size_t>(max_ascii_line_bytes_));
    epoch_deduplicator_ = std::make_unique<um982_raw_driver::EpochDeduplicator>(
      static_cast<std::size_t>(epoch_dedup_capacity_));

    ntrip_enabled_ = declare_parameter<bool>("ntrip.enabled", false);
    ntrip_host_ = declare_parameter<std::string>("ntrip.host", "");
    ntrip_port_ = declare_parameter<int>("ntrip.port", 2101);
    ntrip_mountpoint_ = declare_parameter<std::string>("ntrip.mountpoint", "");
    ntrip_username_ = declare_parameter<std::string>("ntrip.username", "");
    const auto ntrip_password = declare_parameter<std::string>("ntrip.password", "");
    const auto ntrip_password_env =
      declare_parameter<std::string>("ntrip.password_env", "NTRIP_PASSWORD");
    ntrip_password_ = readPasswordFromEnv(ntrip_password, ntrip_password_env);
    ntrip_send_gga_interval_s_ =
      declare_parameter<double>("ntrip.send_gga_interval_s", 5.0);
    ntrip_connect_requires_valid_gga_ =
      declare_parameter<bool>("ntrip.connect_requires_valid_gga", true);

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
    heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("/heading", 10);
    raw_pub_ = create_publisher<nmea_msgs::msg::Sentence>("rtk/nmea_sentence", 50);
    status_pub_ = create_publisher<std_msgs::msg::String>("rtk/status", 10);
    raw_frame_pub_ =
      create_publisher<gnss_raw_msgs::msg::RawFrame>(raw_output_topic_, 100);
    observation_pub_ =
      create_publisher<gnss_raw_msgs::msg::ObservationEpoch>(observation_topic_, 50);
    ephemeris_pub_ =
      create_publisher<gnss_raw_msgs::msg::Ephemeris>(ephemeris_topic_, 50);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);

    if (ntrip_enabled_) {
      validateNtripParameters();
    }

    running_ = true;
    tryOpenSerial();
    serial_thread_ = std::thread(&Um982RtkNode::serialLoop, this);
    if (ntrip_enabled_) {
      ntrip_thread_ = std::thread(&Um982RtkNode::ntripLoop, this);
    }

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(status_period_s_ * 1000.0)),
      [this]() {
        publishStatus();
        publishDiagnostics();
      });
  }

  ~Um982RtkNode() override
  {
    running_ = false;
    if (ntrip_socket_ >= 0) {
      ::shutdown(ntrip_socket_, SHUT_RDWR);
      ::close(ntrip_socket_);
      ntrip_socket_ = -1;
    }
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
    if (ntrip_thread_.joinable()) {
      ntrip_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      closeSerialLocked();
    }
  }

private:
  void validateStreamParameters() const
  {
    if (stream_mode_ != "nmea_only" && stream_mode_ != "mixed") {
      throw std::invalid_argument("stream_mode must be 'nmea_only' or 'mixed'");
    }
    if (baud_ <= 0 || read_chunk_bytes_ <= 0 || max_read_batches_ <= 0 ||
      read_timeout_ms_ < 0 || max_payload_bytes_ < 0 || max_payload_bytes_ > 65535 ||
      max_buffer_bytes_ <= 0 || max_ascii_line_bytes_ <= 0 ||
      read_chunk_bytes_ > max_buffer_bytes_ || max_ascii_line_bytes_ > max_buffer_bytes_ ||
      max_read_batches_ > 1024 || epoch_dedup_capacity_ <= 0 ||
      epoch_dedup_capacity_ > 100000 || reconnect_period_s_ <= 0.0 ||
      stale_ascii_timeout_s_ <= 0.0 ||
      stale_binary_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("UM982 unified stream parameters are outside valid bounds");
    }
  }

  void validateNtripParameters()
  {
    if (ntrip_host_.empty() || ntrip_mountpoint_.empty() || ntrip_username_.empty() ||
      ntrip_password_.empty())
    {
      throw std::runtime_error(
              "NTRIP is enabled but host, mountpoint, username, or password is empty");
    }
  }

  bool tryOpenSerial()
  {
    if (serial_connected_.load()) {
      return true;
    }
    if (std::chrono::steady_clock::now() < next_reconnect_) {
      return false;
    }

    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      closeSerialLocked();
      serial_port_.setPort(port_);
      serial_port_.setBaudrate(static_cast<std::uint32_t>(baud_));
      serial::Timeout timeout =
        serial::Timeout::simpleTimeout(static_cast<std::uint32_t>(read_timeout_ms_));
      serial_port_.setTimeout(timeout);
      serial_port_.open();
      if (!serial_port_.isOpen()) {
        throw std::runtime_error("serial library returned a closed port after open");
      }
      serial_connected_ = true;
      last_serial_error_.clear();
      RCLCPP_INFO(
        get_logger(), "UM982 unified serial opened: %s @ %d (%s)", port_.c_str(), baud_,
        stream_mode_.c_str());
      return true;
    } catch (const std::exception & error) {
      markDisconnected(error.what());
      return false;
    }
  }

  void closeSerialLocked() noexcept
  {
    try {
      if (serial_port_.isOpen()) {
        serial_port_.close();
      }
    } catch (const std::exception &) {
    }
    serial_connected_ = false;
  }

  void markDisconnected(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      closeSerialLocked();
      last_serial_error_ = reason;
    }
    {
      std::lock_guard<std::mutex> lock(framer_mutex_);
      mixed_framer_->reset();
    }
    {
      std::lock_guard<std::mutex> lock(ascii_status_mutex_);
      have_valid_ascii_sentence_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(raw_status_mutex_);
      epoch_deduplicator_->reset();
      have_valid_binary_frame_ = false;
    }
    next_reconnect_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_period_s_));
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UM982 unified port unavailable: %s", reason.c_str());
  }

  void serialLoop()
  {
    while (rclcpp::ok() && running_) {
      if (!tryOpenSerial()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      try {
        bool received_any = false;
        for (int batch = 0; batch < max_read_batches_; ++batch) {
          std::vector<std::uint8_t> chunk;
          {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            const std::size_t available = serial_port_.available();
            if (available == 0U) {
              break;
            }
            const std::size_t requested = std::min(
              available, static_cast<std::size_t>(read_chunk_bytes_));
            const std::size_t received = serial_port_.read(chunk, requested);
            chunk.resize(received);
          }
          if (chunk.empty()) {
            break;
          }
          received_any = true;
          std::vector<um982_raw_driver::MixedStreamItem> items;
          {
            std::lock_guard<std::mutex> lock(framer_mutex_);
            items = mixed_framer_->consume(chunk);
          }
          for (const auto & item : items) {
            if (item.kind == um982_raw_driver::MixedStreamItemKind::AsciiLine) {
              handleLine(item.ascii_line);
            } else if (mixed_mode_) {
              publishRawFrame(item.binary_frame, now());
            } else {
              unexpected_binary_frames_.fetch_add(1U);
            }
          }
        }
        if (!received_any) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      } catch (const std::exception & exc) {
        if (running_) {
          markDisconnected(exc.what());
        }
      }
    }
  }

  void handleLine(const std::string & line)
  {
    const auto parsed = parseSentence(line);
    if (!parsed.has_value()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(ascii_status_mutex_);
      have_valid_ascii_sentence_ = true;
      last_valid_ascii_sentence_ = std::chrono::steady_clock::now();
    }

    const auto stamp = now();
    if (publish_raw_) {
      nmea_msgs::msg::Sentence raw;
      raw.header.stamp = toRosTimeMsg(stamp);
      raw.header.frame_id = frame_id_;
      raw.sentence = line;
      raw_pub_->publish(raw);
    }

    if (parsed->gga.has_value()) {
      {
        std::lock_guard<std::mutex> lock(gga_mutex_);
        latest_gga_sentence_ = line;
        latest_gga_valid_for_ntrip_ = parsed->gga->fix_quality != 0;
      }
      publishFix(*parsed->gga, stamp);
    }
    if (parsed->rmc.has_value()) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_rmc_valid_ = parsed->rmc->valid;
      last_speed_mps_ = parsed->rmc->speed_mps;
      last_course_deg_ = parsed->rmc->course_deg;
    }
    if (parsed->ths.has_value()) {
      if (parsed->ths->mode != "V") {
        publishHeading(parsed->ths->heading_deg, stamp);
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_heading_deg_ = parsed->ths->heading_deg;
        last_heading_source_ = "THS";
        last_heading_valid_ = true;
      }
    }
    if (parsed->hpr.has_value()) {
      if (parsed->hpr->fix_type > 0) {
        publishHeading(parsed->hpr->heading_deg, stamp);
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_heading_deg_ = parsed->hpr->heading_deg;
        last_heading_source_ = "HPR";
        last_heading_valid_ = true;
      }
    }
    if (parsed->uniheading.has_value()) {
      const auto & data = *parsed->uniheading;
      const bool heading_valid =
        data.solution_status == "SOL_COMPUTED" && data.position_type != "NONE";
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_uniheading_status_ = data.solution_status + "/" + data.position_type;
      }
      if (heading_valid) {
        publishHeading(data.heading_deg, stamp);
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_heading_deg_ = data.heading_deg;
        last_heading_source_ = "UNIHEADING";
        last_heading_valid_ = true;
      }
    }
  }

  void publishFix(const GgaData & data, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = toRosTimeMsg(stamp);
    fix.header.frame_id = frame_id_;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    const double default_epe = defaultEpeForQuality(data.fix_quality);
    switch (data.fix_quality) {
      case 1:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        break;
      case 2:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        break;
      case 4:
      case 5:
      case 9:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        break;
      default:
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        break;
    }

    fix.latitude = data.latitude;
    fix.longitude = data.longitude;
    fix.altitude = data.altitude + data.mean_sea_level;
    const double hdop = std::isfinite(data.hdop) ? data.hdop : 1.0;
    fix.position_covariance[0] = std::pow(hdop * default_epe, 2);
    fix.position_covariance[4] = std::pow(hdop * default_epe, 2);
    fix.position_covariance[8] = std::pow(2.0 * hdop * default_epe * 2.0, 2);
    fix_pub_->publish(fix);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_fix_quality_ = data.fix_quality;
      last_satellites_ = data.satellites;
      last_hdop_ = data.hdop;
      last_latitude_ = data.latitude;
      last_longitude_ = data.longitude;
      last_altitude_ = fix.altitude;
    }
  }

  void publishHeading(double heading_deg, const rclcpp::Time & stamp)
  {
    const double calibrated_heading_deg =
      applyHeadingOffsetDeg(heading_deg, heading_offset_deg_);
    if (!std::isfinite(calibrated_heading_deg)) {
      return;
    }
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = toRosTimeMsg(stamp);
    msg.header.frame_id = frame_id_;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, calibrated_heading_deg * kPi / 180.0);
    msg.quaternion.x = quaternion.x();
    msg.quaternion.y = quaternion.y();
    msg.quaternion.z = quaternion.z();
    msg.quaternion.w = quaternion.w();
    heading_pub_->publish(msg);
  }

  double defaultEpeForQuality(int quality) const
  {
    switch (quality) {
      case 1:
        return epe_quality_1_;
      case 2:
        return epe_quality_2_;
      case 4:
        return epe_quality_4_;
      case 5:
        return epe_quality_5_;
      case 9:
        return epe_quality_9_;
      default:
        return epe_quality_0_;
    }
  }

  void publishRawFrame(
    const um982_raw_driver::BinaryFrame & frame,
    const rclcpp::Time & reception_stamp)
  {
    std::lock_guard<std::mutex> lock(raw_status_mutex_);
    const std::uint64_t current_frame_index = frame_index_++;
    gnss_raw_msgs::msg::RawFrame message;
    message.header.stamp = reception_stamp;
    message.header.frame_id = raw_frame_id_;
    message.source_port = port_;
    message.frame_index = current_frame_index;
    message.stream_offset = frame.stream_offset;
    message.cpu_idle_percent = frame.header.cpu_idle_percent;
    message.message_id = frame.header.message_id;
    message.payload_length = frame.header.payload_length;
    message.time_reference = frame.header.time_reference;
    message.time_status = frame.header.time_status;
    message.week = frame.header.week;
    message.milliseconds_of_week = frame.header.milliseconds_of_week;
    message.format_version = frame.header.format_version;
    message.leap_seconds = frame.header.leap_seconds;
    message.output_delay_ms = frame.header.output_delay_ms;
    message.crc_valid = true;
    message.data = frame.bytes;
    raw_frame_pub_->publish(std::move(message));
    publishObservationEpoch(frame, current_frame_index, reception_stamp);
    publishEphemeris(frame, current_frame_index, reception_stamp);
    have_valid_binary_frame_ = true;
    last_valid_binary_frame_ = std::chrono::steady_clock::now();
  }

  void publishObservationEpoch(
    const um982_raw_driver::BinaryFrame & frame,
    const std::uint64_t current_frame_index,
    const rclcpp::Time & reception_stamp)
  {
    if (!um982_raw_driver::isObservationMessage(frame.header.message_id)) {
      return;
    }
    ++observation_frames_seen_;
    const auto decoded = um982_raw_driver::decodeObservationFrame(frame);
    if (!decoded.ok()) {
      ++observation_decode_failures_;
      last_observation_decode_error_ = decoded.reason;
      return;
    }
    const um982_raw_driver::ObservationEpochKey epoch_key{
      decoded.epoch->receiver,
      frame.header.time_reference,
      frame.header.week,
      frame.header.milliseconds_of_week};
    if (!epoch_deduplicator_->accept(epoch_key)) {
      ++duplicate_observation_epochs_;
      return;
    }

    gnss_raw_msgs::msg::ObservationEpoch message;
    message.header.stamp = reception_stamp;
    message.header.frame_id = raw_frame_id_;
    message.source_port = port_;
    message.frame_index = current_frame_index;
    message.stream_offset = frame.stream_offset;
    message.receiver = static_cast<std::uint8_t>(decoded.epoch->receiver);
    message.source_message_id = frame.header.message_id;
    message.time_reference = frame.header.time_reference;
    message.time_status = frame.header.time_status;
    message.week = frame.header.week;
    message.milliseconds_of_week = frame.header.milliseconds_of_week;
    message.output_delay_ms = frame.header.output_delay_ms;
    message.observations.reserve(decoded.epoch->observations.size());
    for (const auto & source : decoded.epoch->observations) {
      gnss_raw_msgs::msg::Observation observation;
      observation.constellation = static_cast<std::uint8_t>(source.constellation);
      observation.prn = source.prn;
      observation.signal_type = source.signal_type;
      observation.channel_number = source.channel_number;
      observation.system_frequency = source.system_frequency;
      observation.glonass_frequency_channel = source.glonass_frequency_channel;
      observation.pseudorange_m = source.pseudorange_m;
      observation.carrier_phase_cycles = source.carrier_phase_cycles;
      observation.doppler_hz = source.doppler_hz;
      observation.pseudorange_std_m = source.pseudorange_std_m;
      observation.carrier_phase_std_cycles = source.carrier_phase_std_cycles;
      observation.cn0_db_hz = source.cn0_db_hz;
      observation.lock_time_s = source.lock_time_s;
      observation.pseudorange_valid = source.pseudorange_valid;
      observation.carrier_phase_valid = source.carrier_phase_valid;
      observation.l2c_signal = source.l2c_signal;
      observation.tracking_status = source.tracking_status;
      message.observations.push_back(std::move(observation));
    }
    observations_published_ += message.observations.size();
    ++observation_epochs_published_;
    observation_pub_->publish(std::move(message));
  }

  void publishEphemeris(
    const um982_raw_driver::BinaryFrame & frame,
    const std::uint64_t current_frame_index,
    const rclcpp::Time & reception_stamp)
  {
    if (!um982_raw_driver::isEphemerisMessage(frame.header.message_id)) {
      return;
    }
    ++ephemeris_frames_seen_;
    const auto decoded = um982_raw_driver::decodeEphemerisFrame(frame);
    if (!decoded.ok()) {
      ++ephemeris_decode_failures_;
      last_ephemeris_decode_error_ = decoded.reason;
      return;
    }
    const auto & source = *decoded.ephemeris;
    gnss_raw_msgs::msg::Ephemeris message;
    message.header.stamp = reception_stamp;
    message.header.frame_id = raw_frame_id_;
    message.source_port = port_;
    message.frame_index = current_frame_index;
    message.stream_offset = frame.stream_offset;
    message.source_message_id = frame.header.message_id;
    message.time_reference = frame.header.time_reference;
    message.time_status = frame.header.time_status;
    message.header_week = frame.header.week;
    message.header_milliseconds_of_week = frame.header.milliseconds_of_week;
    message.constellation = static_cast<std::uint8_t>(source.constellation);
    message.prn = source.prn;
    message.model = static_cast<std::uint8_t>(source.model);
    message.reference_frame = static_cast<std::uint8_t>(source.reference_frame);
    message.health = source.health;
    message.issue_of_data_ephemeris = source.issue_of_data_ephemeris;
    message.issue_of_data_clock = source.issue_of_data_clock;
    message.week = source.week;
    message.toe_s = source.toe_s;
    message.toc_s = source.toc_s;
    message.semi_major_axis_m = source.semi_major_axis_m;
    message.delta_mean_motion_rad_s = source.delta_mean_motion_rad_s;
    message.mean_anomaly_rad = source.mean_anomaly_rad;
    message.eccentricity = source.eccentricity;
    message.argument_of_perigee_rad = source.argument_of_perigee_rad;
    message.cuc_rad = source.cuc_rad;
    message.cus_rad = source.cus_rad;
    message.crc_m = source.crc_m;
    message.crs_m = source.crs_m;
    message.cic_rad = source.cic_rad;
    message.cis_rad = source.cis_rad;
    message.inclination_rad = source.inclination_rad;
    message.inclination_rate_rad_s = source.inclination_rate_rad_s;
    message.ascending_node_rad = source.ascending_node_rad;
    message.ascending_node_rate_rad_s = source.ascending_node_rate_rad_s;
    message.clock_bias_s = source.clock_bias_s;
    message.clock_drift_s_s = source.clock_drift_s_s;
    message.clock_drift_rate_s_s2 = source.clock_drift_rate_s_s2;
    message.group_delay_1_s = source.group_delay_1_s;
    message.group_delay_2_s = source.group_delay_2_s;
    message.group_delay_1_valid = source.group_delay_1_valid;
    message.group_delay_2_valid = source.group_delay_2_valid;
    message.corrected_mean_motion_rad_s = source.corrected_mean_motion_rad_s;
    message.ura_variance_m2 = source.ura_variance_m2;
    message.ura_variance_valid = source.ura_variance_valid;
    message.accuracy_index = source.accuracy_index;
    message.glonass_frequency_channel = source.glonass_frequency_channel;
    message.position_ecef_m = source.position_ecef_m;
    message.velocity_ecef_m_s = source.velocity_ecef_m_s;
    message.acceleration_ecef_m_s2 = source.acceleration_ecef_m_s2;
    message.glonass_clock_bias_s = source.glonass_clock_bias_s;
    message.glonass_relative_frequency_bias = source.glonass_relative_frequency_bias;
    message.glonass_l1_l2_delay_s = source.glonass_l1_l2_delay_s;
    message.glonass_frame_time_s = source.glonass_frame_time_s;
    message.glonass_flags = source.glonass_flags;
    ++ephemerides_published_;
    ephemeris_pub_->publish(std::move(message));
  }

  void publishStatus()
  {
    std_msgs::msg::String msg;
    std::ostringstream status;
    std::lock_guard<std::mutex> lock(status_mutex_);
    status << "fix=" << fixQualityText(last_fix_quality_)
           << " q=" << last_fix_quality_
           << " sats=" << last_satellites_
           << " hdop=" << last_hdop_
           << " lat=" << std::fixed << std::setprecision(8) << last_latitude_
           << " lon=" << last_longitude_
           << " alt=" << std::setprecision(3) << last_altitude_
           << " heading=";
    if (last_heading_valid_) {
      status << std::setprecision(3)
             << applyHeadingOffsetDeg(last_heading_deg_, heading_offset_deg_)
             << " " << last_heading_source_
             << " raw=" << last_heading_deg_
             << " offset=" << heading_offset_deg_;
    } else {
      status << "-";
    }
    status << " rmc=" << (last_rmc_valid_ ? "A" : "V")
           << " speed_mps=" << std::setprecision(2) << last_speed_mps_
           << " cog=" << last_course_deg_
           << " uniheading=" << last_uniheading_status_
           << " ntrip=" << (ntrip_connected_.load() ? "connected" : "offline")
           << " rtcm_bytes=" << rtcm_bytes_.load()
           << " stream_mode=" << stream_mode_
           << " unexpected_binary=" << unexpected_binary_frames_.load();
    msg.data = status.str();
    status_pub_->publish(msg);
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "um982_rtk_driver/unified_stream";
    status.hardware_id = port_;

    um982_raw_driver::MixedStreamFramerStats framer_stats;
    std::size_t buffered_bytes = 0U;
    {
      std::lock_guard<std::mutex> lock(framer_mutex_);
      framer_stats = mixed_framer_->stats();
      buffered_bytes = mixed_framer_->bufferedBytes();
    }

    bool have_ascii = false;
    double ascii_age_s = -1.0;
    {
      std::lock_guard<std::mutex> lock(ascii_status_mutex_);
      have_ascii = have_valid_ascii_sentence_;
      if (have_ascii) {
        ascii_age_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_valid_ascii_sentence_).count();
      }
    }

    std::string last_serial_error;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      last_serial_error = last_serial_error_;
    }

    std::lock_guard<std::mutex> raw_lock(raw_status_mutex_);
    const double binary_age_s = have_valid_binary_frame_ ?
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_valid_binary_frame_).count() :
      -1.0;
    if (!serial_connected_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "SERIAL_DISCONNECTED";
    } else if (!have_ascii || ascii_age_s > stale_ascii_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "NO_RECENT_VALID_ASCII";
    } else if (mixed_mode_ &&
      (!have_valid_binary_frame_ || binary_age_s > stale_binary_timeout_s_))
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "NO_RECENT_VALID_BINARY_FRAME";
    } else if (observation_decode_failures_ != 0U || ephemeris_decode_failures_ != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "CANONICAL_DECODE_FAILURE";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = mixed_mode_ ? "MIXED_STREAMING" : "NMEA_ONLY_STREAMING";
    }

    status.values.push_back(keyValue("stream_mode", stream_mode_));
    status.values.push_back(
      keyValue("serial_connected", serial_connected_.load() ? "true" : "false"));
    status.values.push_back(keyValue("last_serial_error", last_serial_error));
    status.values.push_back(numericKeyValue("baud", baud_));
    status.values.push_back(numericKeyValue("ascii_age_s", ascii_age_s));
    status.values.push_back(numericKeyValue("binary_age_s", binary_age_s));
    status.values.push_back(numericKeyValue("bytes_received", framer_stats.bytes_received));
    status.values.push_back(numericKeyValue("bytes_discarded", framer_stats.bytes_discarded));
    status.values.push_back(
      numericKeyValue("ascii_lines_emitted", framer_stats.ascii_lines_emitted));
    status.values.push_back(
      numericKeyValue("binary_frames_emitted", framer_stats.binary_frames_emitted));
    status.values.push_back(numericKeyValue("crc_failures", framer_stats.crc_failures));
    status.values.push_back(numericKeyValue("length_failures", framer_stats.length_failures));
    status.values.push_back(numericKeyValue("ascii_overflows", framer_stats.ascii_overflows));
    status.values.push_back(
      numericKeyValue("invalid_ascii_lines", framer_stats.invalid_ascii_lines));
    status.values.push_back(
      numericKeyValue("buffer_overflows", framer_stats.buffer_overflows));
    status.values.push_back(numericKeyValue("buffered_bytes", buffered_bytes));
    status.values.push_back(
      numericKeyValue("unexpected_binary_frames", unexpected_binary_frames_.load()));
    status.values.push_back(
      numericKeyValue("observation_frames_seen", observation_frames_seen_));
    status.values.push_back(
      numericKeyValue("observation_epochs_published", observation_epochs_published_));
    status.values.push_back(
      numericKeyValue("observations_published", observations_published_));
    status.values.push_back(
      numericKeyValue("observation_decode_failures", observation_decode_failures_));
    status.values.push_back(
      numericKeyValue("duplicate_observation_epochs", duplicate_observation_epochs_));
    status.values.push_back(
      numericKeyValue("epoch_dedup_entries", epoch_deduplicator_->size()));
    status.values.push_back(
      keyValue("last_observation_decode_error", last_observation_decode_error_));
    status.values.push_back(
      numericKeyValue("ephemeris_frames_seen", ephemeris_frames_seen_));
    status.values.push_back(
      numericKeyValue("ephemerides_published", ephemerides_published_));
    status.values.push_back(
      numericKeyValue("ephemeris_decode_failures", ephemeris_decode_failures_));
    status.values.push_back(
      keyValue("last_ephemeris_decode_error", last_ephemeris_decode_error_));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  void ntripLoop()
  {
    while (rclcpp::ok() && running_) {
      if (!ntrip_connected_) {
        std::string gga;
        bool valid = false;
        {
          std::lock_guard<std::mutex> lock(gga_mutex_);
          gga = latest_gga_sentence_;
          valid = latest_gga_valid_for_ntrip_;
        }
        if (!gga.empty() && (!ntrip_connect_requires_valid_gga_ || valid)) {
          connectNtrip(gga);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          continue;
        }
      }
      pumpNtrip();
    }
  }

  void connectNtrip(const std::string & initial_gga)
  {
    ntrip_socket_ = openSocket();
    if (ntrip_socket_ < 0) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      return;
    }

    std::string mountpoint = ntrip_mountpoint_;
    if (!mountpoint.empty() && mountpoint.front() != '/') {
      mountpoint = "/" + mountpoint;
    }
    const std::string token = base64Encode(ntrip_username_ + ":" + ntrip_password_);
    std::ostringstream request;
    request << "GET " << mountpoint << " HTTP/1.1\r\n"
            << "Host: " << ntrip_host_ << ":" << ntrip_port_ << "\r\n"
            << "Ntrip-Version: Ntrip/2.0\r\n"
            << "User-Agent: XJTLU-UM982-RTK/1.0\r\n"
            << "Authorization: Basic " << token << "\r\n"
            << "Connection: close\r\n\r\n";
    const auto request_text = request.str();
    ::send(ntrip_socket_, request_text.data(), request_text.size(), 0);

    std::string header;
    auto response_state = NtripResponseState::NeedMore;
    char c = 0;
    while (running_ && response_state == NtripResponseState::NeedMore) {
      const auto n = ::recv(ntrip_socket_, &c, 1, 0);
      if (n <= 0) {
        closeNtrip();
        return;
      }
      header.push_back(c);
      response_state = evaluateNtripResponse(header);
      if (header.size() > 4096) {
        closeNtrip();
        return;
      }
    }

    if (response_state != NtripResponseState::Accepted) {
      RCLCPP_WARN(get_logger(), "NTRIP rejected connection: %.120s", header.c_str());
      closeNtrip();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      return;
    }

    ntrip_connected_ = true;
    sendGgaToNtrip(initial_gga);
    last_gga_sent_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "NTRIP connected to %s:%d%s",
      ntrip_host_.c_str(), ntrip_port_, mountpoint.c_str());
  }

  int openSocket()
  {
    addrinfo hints {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * result = nullptr;
    const std::string port_text = std::to_string(ntrip_port_);
    const int rc = ::getaddrinfo(ntrip_host_.c_str(), port_text.c_str(), &hints, &result);
    if (rc != 0) {
      RCLCPP_WARN(
        get_logger(), "NTRIP DNS failed for %s: %s", ntrip_host_.c_str(),
        gai_strerror(rc));
      return -1;
    }

    int fd = -1;
    for (addrinfo * rp = result; rp != nullptr; rp = rp->ai_next) {
      fd = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (fd < 0) {
        continue;
      }
      if (::connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
        break;
      }
      ::close(fd);
      fd = -1;
    }
    ::freeaddrinfo(result);
    return fd;
  }

  void pumpNtrip()
  {
    const auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_time - last_gga_sent_).count() >=
      ntrip_send_gga_interval_s_)
    {
      std::string gga;
      {
        std::lock_guard<std::mutex> lock(gga_mutex_);
        gga = latest_gga_sentence_;
      }
      if (!gga.empty()) {
        sendGgaToNtrip(gga);
        last_gga_sent_ = now_time;
      }
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ntrip_socket_, &readfds);
    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    const int ready = ::select(ntrip_socket_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return;
    }

    std::vector<uint8_t> buffer(8192);
    const ssize_t bytes = ::recv(ntrip_socket_, buffer.data(), buffer.size(), 0);
    if (bytes <= 0) {
      RCLCPP_WARN(get_logger(), "NTRIP connection closed");
      closeNtrip();
      return;
    }

    try {
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_port_.write(buffer.data(), static_cast<size_t>(bytes));
      }
      rtcm_bytes_.fetch_add(static_cast<uint64_t>(bytes));
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to write RTCM to UM982: %s", exc.what());
    }
  }

  void sendGgaToNtrip(const std::string & gga)
  {
    if (ntrip_socket_ < 0) {
      return;
    }
    std::string payload = gga;
    if (payload.find("\r\n") == std::string::npos) {
      payload += "\r\n";
    }
    ::send(ntrip_socket_, payload.data(), payload.size(), 0);
  }

  void closeNtrip()
  {
    if (ntrip_socket_ >= 0) {
      ::close(ntrip_socket_);
      ntrip_socket_ = -1;
    }
    ntrip_connected_ = false;
  }

  serial::Serial serial_port_;
  std::mutex serial_mutex_;
  std::atomic<bool> serial_connected_{false};
  std::string last_serial_error_;
  std::chrono::steady_clock::time_point next_reconnect_{};
  std::thread serial_thread_;
  std::thread ntrip_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<gnss_raw_msgs::msg::RawFrame>::SharedPtr raw_frame_pub_;
  rclcpp::Publisher<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr observation_pub_;
  rclcpp::Publisher<gnss_raw_msgs::msg::Ephemeris>::SharedPtr ephemeris_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string port_;
  int baud_ = 115200;
  std::string frame_id_;
  bool publish_raw_ = true;
  double status_period_s_ = 1.0;
  double heading_offset_deg_ = 0.0;
  double epe_quality_0_ = 1000000.0;
  double epe_quality_1_ = 4.0;
  double epe_quality_2_ = 0.1;
  double epe_quality_4_ = 0.02;
  double epe_quality_5_ = 4.0;
  double epe_quality_9_ = 3.0;

  std::string stream_mode_ = "nmea_only";
  bool mixed_mode_ = false;
  int read_chunk_bytes_ = 4096;
  int max_read_batches_ = 16;
  int read_timeout_ms_ = 20;
  int max_payload_bytes_ = 65535;
  int max_buffer_bytes_ = 131072;
  int max_ascii_line_bytes_ = 1024;
  double reconnect_period_s_ = 1.0;
  std::string raw_frame_id_ = "gnss_raw";
  std::string raw_output_topic_ = "/gnss/raw/frame";
  std::string observation_topic_ = "/gnss/raw/observation_epoch";
  std::string ephemeris_topic_ = "/gnss/raw/ephemeris";
  std::string diagnostics_topic_ = "/gnss/raw/diagnostics";
  int epoch_dedup_capacity_ = 256;
  double stale_ascii_timeout_s_ = 2.0;
  double stale_binary_timeout_s_ = 2.0;
  std::unique_ptr<um982_raw_driver::MixedStreamFramer> mixed_framer_;
  std::unique_ptr<um982_raw_driver::EpochDeduplicator> epoch_deduplicator_;
  std::mutex framer_mutex_;
  std::mutex ascii_status_mutex_;
  bool have_valid_ascii_sentence_ = false;
  std::chrono::steady_clock::time_point last_valid_ascii_sentence_{};
  std::atomic<std::uint64_t> unexpected_binary_frames_{0};

  std::mutex raw_status_mutex_;
  bool have_valid_binary_frame_ = false;
  std::chrono::steady_clock::time_point last_valid_binary_frame_{};
  std::uint64_t frame_index_ = 0;
  std::uint64_t observation_frames_seen_ = 0;
  std::uint64_t observation_epochs_published_ = 0;
  std::uint64_t observations_published_ = 0;
  std::uint64_t observation_decode_failures_ = 0;
  std::uint64_t duplicate_observation_epochs_ = 0;
  std::string last_observation_decode_error_;
  std::uint64_t ephemeris_frames_seen_ = 0;
  std::uint64_t ephemerides_published_ = 0;
  std::uint64_t ephemeris_decode_failures_ = 0;
  std::string last_ephemeris_decode_error_;

  bool ntrip_enabled_ = false;
  std::string ntrip_host_;
  int ntrip_port_ = 2101;
  std::string ntrip_mountpoint_;
  std::string ntrip_username_;
  std::string ntrip_password_;
  double ntrip_send_gga_interval_s_ = 5.0;
  bool ntrip_connect_requires_valid_gga_ = true;
  int ntrip_socket_ = -1;
  std::atomic<bool> ntrip_connected_{false};
  std::chrono::steady_clock::time_point last_gga_sent_ = std::chrono::steady_clock::now();
  std::atomic<uint64_t> rtcm_bytes_{0};

  std::mutex gga_mutex_;
  std::string latest_gga_sentence_;
  bool latest_gga_valid_for_ntrip_ = false;

  std::mutex status_mutex_;
  int last_fix_quality_ = 0;
  int last_satellites_ = 0;
  double last_hdop_ = 0.0;
  double last_latitude_ = 0.0;
  double last_longitude_ = 0.0;
  double last_altitude_ = 0.0;
  bool last_rmc_valid_ = false;
  double last_speed_mps_ = 0.0;
  double last_course_deg_ = 0.0;
  bool last_heading_valid_ = false;
  double last_heading_deg_ = 0.0;
  std::string last_heading_source_;
  std::string last_uniheading_status_ = "-";
};

}  // namespace um982_rtk_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<um982_rtk_driver::Um982RtkNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("um982_rtk_driver"), "Node failed: %s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
