#include "um982_rtk_driver/heading_selector.hpp"
#include "um982_rtk_driver/nmea_parser.hpp"
#include "um982_rtk_driver/ntrip_response.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

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
#include <geometry_msgs/msg/quaternion_stamped.hpp>
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
    const auto primary_heading_source = declare_parameter<std::string>(
      "heading_primary_source", "THS");
    const auto fallback_heading_source = declare_parameter<std::string>(
      "heading_fallback_source", "UNIHEADING");
    const auto primary_source = parseHeadingSource(primary_heading_source);
    const auto fallback_source = parseHeadingSource(fallback_heading_source);
    if (!primary_source.has_value() || !fallback_source.has_value() ||
      *primary_source == *fallback_source)
    {
      throw std::runtime_error(
              "heading_primary_source and heading_fallback_source must be distinct "
              "THS, HPR, or UNIHEADING values");
    }
    HeadingSelectorConfig heading_selector_config;
    heading_selector_config.primary_source = *primary_source;
    heading_selector_config.fallback_source = *fallback_source;
    heading_selector_config.fallback_timeout_s = declare_parameter<double>(
      "heading_fallback_timeout_s", 1.5);
    heading_selector_config.switch_min_samples = declare_parameter<int>(
      "heading_switch_min_samples", 3);
    heading_selector_config.max_rate_degps = declare_parameter<double>(
      "heading_max_rate_degps", 75.0);
    heading_selector_config.max_step_deg = declare_parameter<double>(
      "heading_max_step_deg", 15.0);
    heading_selector_config.rate_slack_deg = declare_parameter<double>(
      "heading_rate_slack_deg", 2.0);
    heading_selector_ = std::make_unique<HeadingSelector>(heading_selector_config);
    epe_quality_0_ = declare_parameter<double>("epe_quality0", 1000000.0);
    epe_quality_1_ = declare_parameter<double>("epe_quality1", 4.0);
    epe_quality_2_ = declare_parameter<double>("epe_quality2", 0.1);
    epe_quality_4_ = declare_parameter<double>("epe_quality4", 0.02);
    epe_quality_5_ = declare_parameter<double>("epe_quality5", 4.0);
    epe_quality_9_ = declare_parameter<double>("epe_quality9", 3.0);

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

    serial_port_.setPort(port_);
    serial_port_.setBaudrate(static_cast<uint32_t>(baud_));
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setTimeout(timeout);
    serial_port_.open();
    if (!serial_port_.isOpen()) {
      throw std::runtime_error("UM982 serial port did not open");
    }

    RCLCPP_INFO(
      get_logger(), "UM982 RTK serial opened: %s @ %d", port_.c_str(), baud_);

    if (ntrip_enabled_) {
      validateNtripParameters();
    }

    running_ = true;
    serial_thread_ = std::thread(&Um982RtkNode::serialLoop, this);
    if (ntrip_enabled_) {
      ntrip_thread_ = std::thread(&Um982RtkNode::ntripLoop, this);
    }

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(status_period_s_ * 1000.0)),
      std::bind(&Um982RtkNode::publishStatus, this));
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
    if (serial_port_.isOpen()) {
      serial_port_.close();
    }
  }

private:
  void validateNtripParameters()
  {
    if (ntrip_host_.empty() || ntrip_mountpoint_.empty() || ntrip_username_.empty() ||
      ntrip_password_.empty())
    {
      throw std::runtime_error(
              "NTRIP is enabled but host, mountpoint, username, or password is empty");
    }
  }

  void serialLoop()
  {
    while (rclcpp::ok() && running_) {
      try {
        std::string line;
        {
          std::lock_guard<std::mutex> lock(serial_mutex_);
          line = serial_port_.readline(512, "\n");
        }
        if (line.empty()) {
          continue;
        }
        handleLine(line);
      } catch (const std::exception & exc) {
        if (running_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "UM982 serial read failed: %s", exc.what());
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
    if (parsed->ths.has_value() && parsed->ths->mode != "V") {
      handleHeading(HeadingSource::THS, parsed->ths->heading_deg, stamp);
    }
    if (parsed->hpr.has_value()) {
      if (parsed->hpr->fix_type > 0) {
        handleHeading(HeadingSource::HPR, parsed->hpr->heading_deg, stamp);
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
        handleHeading(HeadingSource::UNIHEADING, data.heading_deg, stamp);
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

  void handleHeading(
    HeadingSource source, double raw_heading_deg, const rclcpp::Time & stamp)
  {
    const double calibrated_heading_deg =
      applyHeadingOffsetDeg(raw_heading_deg, heading_offset_deg_);
    if (!std::isfinite(calibrated_heading_deg)) {
      return;
    }
    HeadingSelection selection;
    {
      std::lock_guard<std::mutex> lock(heading_mutex_);
      selection = heading_selector_->observe(
        source, calibrated_heading_deg, steadyNowSeconds());
    }
    if (!selection.publish) {
      if (selection.continuity_rejected) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected %s heading jump at %.2f deg", headingSourceName(source),
          calibrated_heading_deg);
      }
      return;
    }
    if (selection.source_switched) {
      RCLCPP_WARN(
        get_logger(), "Switched heading source to %s with %.2f deg continuity bias",
        headingSourceName(source), selection.source_bias_deg);
    }
    publishCalibratedHeading(selection.heading_deg, stamp);
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_heading_deg_ = raw_heading_deg;
    last_heading_calibrated_deg_ = selection.heading_deg;
    last_heading_source_bias_deg_ = selection.source_bias_deg;
    last_heading_source_ = headingSourceName(source);
    last_heading_valid_ = true;
  }

  void publishCalibratedHeading(double calibrated_heading_deg, const rclcpp::Time & stamp)
  {
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

  static double steadyNowSeconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
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
             << last_heading_calibrated_deg_
             << " " << last_heading_source_
             << " raw=" << last_heading_deg_
             << " offset=" << heading_offset_deg_
             << " source_bias=" << last_heading_source_bias_deg_;
    } else {
      status << "-";
    }
    status << " rmc=" << (last_rmc_valid_ ? "A" : "V")
           << " speed_mps=" << std::setprecision(2) << last_speed_mps_
           << " cog=" << last_course_deg_
           << " uniheading=" << last_uniheading_status_
           << " heading_rejects=" << headingRejectedCount()
           << " ntrip=" << (ntrip_connected_.load() ? "connected" : "offline")
           << " rtcm_bytes=" << rtcm_bytes_.load();
    msg.data = status.str();
    status_pub_->publish(msg);
  }

  int headingRejectedCount()
  {
    std::lock_guard<std::mutex> lock(heading_mutex_);
    return heading_selector_->rejectedCount();
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
  std::thread serial_thread_;
  std::thread ntrip_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
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
  std::mutex heading_mutex_;
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
  double last_heading_calibrated_deg_ = 0.0;
  double last_heading_source_bias_deg_ = 0.0;
  std::string last_heading_source_;
  std::string last_uniheading_status_ = "-";
  std::unique_ptr<HeadingSelector> heading_selector_;
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
