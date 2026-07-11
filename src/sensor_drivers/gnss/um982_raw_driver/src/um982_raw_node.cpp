#include "um982_raw_driver/binary_framer.hpp"
#include "um982_raw_driver/epoch_deduplicator.hpp"
#include "um982_raw_driver/observation_decoder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "gnss_raw_msgs/msg/observation.hpp"
#include "gnss_raw_msgs/msg/observation_epoch.hpp"
#include "gnss_raw_msgs/msg/raw_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

namespace um982_raw_driver
{
namespace
{

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

class Um982RawNode : public rclcpp::Node
{
public:
  Um982RawNode()
  : Node("um982_raw_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/rtk_um982_raw");
    baud_ = declare_parameter<int>("baud", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/gnss/raw/frame");
    observation_topic_ =
      declare_parameter<std::string>("observation_topic", "/gnss/raw/observation_epoch");
    diagnostics_topic_ =
      declare_parameter<std::string>("diagnostics_topic", "/gnss/raw/diagnostics");
    read_chunk_bytes_ = declare_parameter<int>("read_chunk_bytes", 4096);
    max_read_batches_ = declare_parameter<int>("max_read_batches_per_tick", 16);
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 20);
    max_payload_bytes_ = declare_parameter<int>("max_payload_bytes", 65535);
    max_buffer_bytes_ = declare_parameter<int>("max_buffer_bytes", 131072);
    epoch_dedup_capacity_ = declare_parameter<int>("epoch_dedup_capacity", 256);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 1.0);
    diagnostics_period_s_ = declare_parameter<double>("diagnostics_period_s", 1.0);
    stale_frame_timeout_s_ = declare_parameter<double>("stale_frame_timeout_s", 2.0);

    validateParameters();
    framer_ = std::make_unique<BinaryFramer>(
      static_cast<std::size_t>(max_payload_bytes_),
      static_cast<std::size_t>(max_buffer_bytes_));
    epoch_deduplicator_ =
      std::make_unique<EpochDeduplicator>(static_cast<std::size_t>(epoch_dedup_capacity_));

    frame_pub_ = create_publisher<gnss_raw_msgs::msg::RawFrame>(output_topic_, 100);
    observation_pub_ =
      create_publisher<gnss_raw_msgs::msg::ObservationEpoch>(observation_topic_, 50);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);

    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&Um982RawNode::readAvailable, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_s_),
      std::bind(&Um982RawNode::publishDiagnostics, this));

    tryOpenSerial();
  }

  ~Um982RawNode() override
  {
    closeSerial();
  }

private:
  void validateParameters() const
  {
    if (port_.empty()) {
      throw std::invalid_argument("port must not be empty");
    }
    if (baud_ <= 0 || read_chunk_bytes_ <= 0 || max_read_batches_ <= 0 ||
      read_timeout_ms_ < 0 || max_payload_bytes_ < 0 || max_payload_bytes_ > 65535 ||
      max_buffer_bytes_ <= 0 || read_chunk_bytes_ > max_buffer_bytes_ ||
      max_read_batches_ > 1024 || epoch_dedup_capacity_ <= 0 ||
      epoch_dedup_capacity_ > 100000 || reconnect_period_s_ <= 0.0 ||
      diagnostics_period_s_ <= 0.0 ||
      stale_frame_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("UM982 raw-driver parameters are outside valid bounds");
    }
  }

  void tryOpenSerial()
  {
    if (serial_connected_) {
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now < next_reconnect_) {
      return;
    }

    try {
      closeSerial();
      serial_port_.setPort(port_);
      serial_port_.setBaudrate(static_cast<std::uint32_t>(baud_));
      serial_port_.setTimeout(
        serial::Timeout::simpleTimeout(static_cast<std::uint32_t>(read_timeout_ms_)));
      serial_port_.open();
      if (!serial_port_.isOpen()) {
        throw std::runtime_error("serial library returned a closed port after open");
      }
      serial_connected_ = true;
      last_serial_error_.clear();
      RCLCPP_INFO(get_logger(), "UM982 raw port opened: %s at %d", port_.c_str(), baud_);
    } catch (const std::exception & error) {
      markDisconnected(error.what());
    }
  }

  void closeSerial() noexcept
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
    closeSerial();
    framer_->reset();
    epoch_deduplicator_->reset();
    last_serial_error_ = reason;
    next_reconnect_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_period_s_));
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "UM982 raw port unavailable: %s", reason.c_str());
  }

  void readAvailable()
  {
    if (!serial_connected_) {
      tryOpenSerial();
      return;
    }

    try {
      for (int batch = 0; batch < max_read_batches_; ++batch) {
        const std::size_t available = serial_port_.available();
        if (available == 0U) {
          break;
        }
        const std::size_t requested = std::min(
          available, static_cast<std::size_t>(read_chunk_bytes_));
        std::vector<std::uint8_t> chunk;
        const std::size_t received = serial_port_.read(chunk, requested);
        if (received == 0U) {
          break;
        }
        chunk.resize(received);
        const auto reception_stamp = now();
        const auto frames = framer_->consume(chunk);
        for (const auto & frame : frames) {
          publishFrame(frame, reception_stamp);
        }
      }
    } catch (const std::exception & error) {
      markDisconnected(error.what());
    }
  }

  void publishFrame(const BinaryFrame & frame, const rclcpp::Time & reception_stamp)
  {
    const std::uint64_t current_frame_index = frame_index_++;
    gnss_raw_msgs::msg::RawFrame message;
    message.header.stamp = reception_stamp;
    message.header.frame_id = frame_id_;
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
    frame_pub_->publish(std::move(message));
    publishObservationEpoch(frame, current_frame_index, reception_stamp);
    have_valid_frame_ = true;
    last_valid_frame_ = std::chrono::steady_clock::now();
  }

  void publishObservationEpoch(
    const BinaryFrame & frame,
    const std::uint64_t current_frame_index,
    const rclcpp::Time & reception_stamp)
  {
    if (!isObservationMessage(frame.header.message_id)) {
      return;
    }
    ++observation_frames_seen_;
    const auto decoded = decodeObservationFrame(frame);
    if (!decoded.ok()) {
      ++observation_decode_failures_;
      last_observation_decode_error_ = decoded.reason;
      return;
    }
    const ObservationEpochKey epoch_key{
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
    message.header.frame_id = frame_id_;
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

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "um982_raw_driver/stream";
    status.hardware_id = port_;
    const double frame_age_s = have_valid_frame_ ?
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_valid_frame_).count() :
      -1.0;

    if (!serial_connected_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "SERIAL_DISCONNECTED";
    } else if (!have_valid_frame_ || frame_age_s > stale_frame_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "NO_RECENT_VALID_FRAME";
    } else if (observation_decode_failures_ != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "OBSERVATION_DECODE_FAILURE";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "STREAMING";
    }

    const auto & stats = framer_->stats();
    status.values.push_back(keyValue("serial_connected", serial_connected_ ? "true" : "false"));
    status.values.push_back(keyValue("last_serial_error", last_serial_error_));
    status.values.push_back(numericKeyValue("baud", baud_));
    status.values.push_back(numericKeyValue("frame_age_s", frame_age_s));
    status.values.push_back(numericKeyValue("bytes_received", stats.bytes_received));
    status.values.push_back(numericKeyValue("bytes_discarded", stats.bytes_discarded));
    status.values.push_back(numericKeyValue("frames_emitted", stats.frames_emitted));
    status.values.push_back(numericKeyValue("crc_failures", stats.crc_failures));
    status.values.push_back(numericKeyValue("length_failures", stats.length_failures));
    status.values.push_back(numericKeyValue("buffer_overflows", stats.buffer_overflows));
    status.values.push_back(numericKeyValue("buffered_bytes", framer_->bufferedBytes()));
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
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  std::string port_;
  int baud_ = 921600;
  std::string frame_id_;
  std::string output_topic_;
  std::string observation_topic_;
  std::string diagnostics_topic_;
  int read_chunk_bytes_ = 4096;
  int max_read_batches_ = 16;
  int read_timeout_ms_ = 20;
  int max_payload_bytes_ = 65535;
  int max_buffer_bytes_ = 131072;
  int epoch_dedup_capacity_ = 256;
  double reconnect_period_s_ = 1.0;
  double diagnostics_period_s_ = 1.0;
  double stale_frame_timeout_s_ = 2.0;

  serial::Serial serial_port_;
  bool serial_connected_ = false;
  std::string last_serial_error_;
  std::chrono::steady_clock::time_point next_reconnect_{};
  bool have_valid_frame_ = false;
  std::chrono::steady_clock::time_point last_valid_frame_{};
  std::uint64_t frame_index_ = 0;
  std::uint64_t observation_frames_seen_ = 0;
  std::uint64_t observation_epochs_published_ = 0;
  std::uint64_t observations_published_ = 0;
  std::uint64_t observation_decode_failures_ = 0;
  std::uint64_t duplicate_observation_epochs_ = 0;
  std::string last_observation_decode_error_;
  std::unique_ptr<BinaryFramer> framer_;
  std::unique_ptr<EpochDeduplicator> epoch_deduplicator_;

  rclcpp::Publisher<gnss_raw_msgs::msg::RawFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<gnss_raw_msgs::msg::ObservationEpoch>::SharedPtr observation_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace um982_raw_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<um982_raw_driver::Um982RawNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("um982_raw_driver"), "Node failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
