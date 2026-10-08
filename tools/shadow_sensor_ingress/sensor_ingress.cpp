// Finite one-way sensor ingress. ROS Humble public serialized callbacks carry
// per-sample writer GIDs; this tool never publishes into its source domain.
#include <algorithm>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>

#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

namespace {
volatile std::sig_atomic_t stop_signal = 0;
void signal_handler(int signum) { stop_signal = signum; }

std::string json_quote(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char byte : value) {
    if (byte == '"' || byte == '\\') out << '\\' << byte;
    else if (byte < 0x20) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
          << static_cast<int>(byte) << std::dec;
    } else out << byte;
  }
  out << '"';
  return out.str();
}

struct Arguments {
  int source_domain, research_domain;
  double duration_s, discovery_delay_s;
  bool audit_payloads;
  std::filesystem::path output;
};

Arguments arguments(int argc, char **argv) {
  std::map<std::string, std::string> values;
  const std::vector<std::string> keys = {
    "--source-domain", "--research-domain", "--duration-s", "--output",
    "--discovery-delay-s", "--audit-payloads"};
  for (int i = 1; i < argc; i += 2) {
    const std::string key = argv[i];
    if (i + 1 >= argc || std::find(keys.begin(), keys.end(), key) == keys.end()
        || values.count(key)) throw std::invalid_argument("unknown, duplicate or incomplete argument");
    values[key] = argv[i + 1];
  }
  for (const auto &key : {"--source-domain", "--research-domain", "--duration-s", "--output"}) {
    if (!values.count(key)) throw std::invalid_argument(std::string("required: ") + key);
  }
  auto integer = [&](const std::string &key) {
    size_t used = 0;
    const int value = std::stoi(values.at(key), &used);
    if (used != values.at(key).size()) throw std::invalid_argument("invalid integer: " + key);
    return value;
  };
  auto number = [&](const std::string &key) {
    size_t used = 0;
    const double value = std::stod(values.at(key), &used);
    if (used != values.at(key).size() || !std::isfinite(value)) {
      throw std::invalid_argument("invalid finite number: " + key);
    }
    return value;
  };
  Arguments args{integer("--source-domain"), integer("--research-domain"),
    number("--duration-s"),
    values.count("--discovery-delay-s") ? number("--discovery-delay-s") : 0.,
    values.count("--audit-payloads") && values.at("--audit-payloads") == "true",
    values.at("--output")};
  if (values.count("--audit-payloads") && values.at("--audit-payloads") != "true"
      && values.at("--audit-payloads") != "false") throw std::invalid_argument("audit-payloads requires true or false");
  const char *localhost = std::getenv("ROS_LOCALHOST_ONLY");
  if (!localhost || std::string(localhost) != "1") {
    throw std::invalid_argument("localhost-only transport required");
  }
  if (args.source_domain < 0 || args.source_domain > 232
      || args.research_domain < 0 || args.research_domain > 232
      || args.source_domain == args.research_domain) {
    throw std::invalid_argument("two distinct explicit ROS domains in 0..232 required");
  }
  if (args.duration_s <= 0. || args.duration_s > 3600.
      || args.discovery_delay_s < 0. || args.discovery_delay_s > 5.
      || args.discovery_delay_s >= args.duration_s) {
    throw std::invalid_argument("duration in (0,3600], discovery delay in [0,min(5,duration)) required");
  }
  if (!args.output.is_absolute()) throw std::invalid_argument("absolute receipt path required");
  return args;
}

class CdrChain {
 public:
  CdrChain() : context_(EVP_MD_CTX_new(), EVP_MD_CTX_free) {
    if (!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1)
      throw std::runtime_error("cannot initialize CDR audit digest");
  }
  void update(const uint8_t *bytes, size_t size) {
    uint8_t length[8];
    for (unsigned i = 0; i < 8; ++i) length[i] = static_cast<uint64_t>(size) >> (8 * i);
    if (EVP_DigestUpdate(context_.get(), length, sizeof(length)) != 1
        || EVP_DigestUpdate(context_.get(), bytes, size) != 1)
      throw std::runtime_error("cannot update CDR audit digest");
  }
  std::string digest() const {
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> copy(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    unsigned char bytes[EVP_MAX_MD_SIZE];
    unsigned size = 0;
    if (!copy || EVP_MD_CTX_copy_ex(copy.get(), context_.get()) != 1
        || EVP_DigestFinal_ex(copy.get(), bytes, &size) != 1)
      throw std::runtime_error("cannot finalize CDR audit digest");
    std::ostringstream out;
    for (unsigned i = 0; i < size; ++i)
      out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(bytes[i]);
    return out.str();
  }
 private:
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_;
};

class SensorIngress {
 public:
  std::map<std::string, size_t> forwarded, denials;
  std::map<std::string, std::string> allowlist;

  explicit SensorIngress(const Arguments &args)
      : source_context_(std::make_shared<rclcpp::Context>()),
        research_context_(std::make_shared<rclcpp::Context>()), audit_payloads_(args.audit_payloads) {
    rclcpp::InitOptions source_init, research_init;
    source_init.set_domain_id(args.source_domain);
    research_init.set_domain_id(args.research_domain);
    research_init.auto_initialize_logging(false);
    source_context_->init(0, nullptr, source_init);
    research_context_->init(0, nullptr, research_init);
    auto options = [](const std::shared_ptr<rclcpp::Context> &context) {
      return rclcpp::NodeOptions().context(context).use_global_arguments(false)
        .enable_rosout(false).start_parameter_services(false)
        .start_parameter_event_publisher(false);
    };
    source_ = std::make_shared<rclcpp::Node>(
      "research_shadow_source_tap", options(source_context_));
    output_ = std::make_shared<rclcpp::Node>(
      "research_shadow_sensor_ingress", options(research_context_));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = source_context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(source_);

    add<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", "livox_ros_driver2/msg/CustomMsg");
    add<sensor_msgs::msg::Imu>("/livox/imu", "sensor_msgs/msg/Imu");
    add<sensor_msgs::msg::NavSatFix>("/fix", "sensor_msgs/msg/NavSatFix");
    add<geometry_msgs::msg::QuaternionStamped>("/heading", "geometry_msgs/msg/QuaternionStamped");
    add<nmea_msgs::msg::Sentence>("/rtk/nmea_sentence", "nmea_msgs/msg/Sentence");
    add<std_msgs::msg::String>("/rtk/status", "std_msgs/msg/String");
    add<diagnostic_msgs::msg::DiagnosticArray>("/rtk/health", "diagnostic_msgs/msg/DiagnosticArray");
  }

  ~SensorIngress() {
    try { close(); } catch (...) {}
  }

  std::map<std::string, std::string> payload_chains() const {
    std::map<std::string, std::string> result;
    for (const auto &[topic, chain] : chains_) result[topic] = chain->digest();
    return result;
  }

  void spin_once() { executor_->spin_once(std::chrono::milliseconds(20)); }

  void close() {
    if (executor_) {
      executor_->remove_node(source_);
      executor_.reset();
    }
    subscriptions_.clear();
    publishers_.clear();
    source_.reset();
    output_.reset();
    if (source_context_ && source_context_->is_valid()) source_context_->shutdown("shadow finished");
    if (research_context_ && research_context_->is_valid()) research_context_->shutdown("shadow finished");
  }

 private:
  template <class MessageT>
  void add(const std::string &topic, const std::string &wire_type) {
    allowlist[topic] = wire_type;
    auto publisher = output_->create_generic_publisher(
      topic, wire_type, rclcpp::QoS(10).reliable().durability_volatile());
    publishers_.push_back(publisher);
    auto callback = [this, topic, wire_type, publisher](
        std::shared_ptr<const rclcpp::SerializedMessage> serialized,
        const rclcpp::MessageInfo &message_info) {
      const auto endpoints = source_->get_publishers_info_by_topic(topic);
      if (endpoints.size() != 1 || endpoints[0].topic_type() != wire_type) {
        ++denials[topic + ":source_owner_or_type"];
        return;
      }
      const auto &sample_gid = message_info.get_rmw_message_info().publisher_gid;
      const auto &discovered_gid = endpoints[0].endpoint_gid();
      const bool populated = std::any_of(
        std::begin(sample_gid.data), std::end(sample_gid.data),
        [](uint8_t byte) { return byte != 0; });
      if (!populated || !std::equal(
          discovered_gid.begin(), discovered_gid.end(), std::begin(sample_gid.data))) {
        ++denials[topic + ":sample_writer"];
        return;
      }
      publisher->publish(*serialized);
      ++forwarded[topic];
      if (audit_payloads_) {
        if (!chains_.count(topic)) chains_[topic] = std::make_unique<CdrChain>();
        chains_[topic]->update(serialized->get_rcl_serialized_message().buffer, serialized->size());
      }
    };
    subscriptions_.push_back(source_->create_subscription<MessageT>(
      topic, rclcpp::SensorDataQoS().keep_last(10), callback));
  }

  std::shared_ptr<rclcpp::Context> source_context_, research_context_;
  bool audit_payloads_;
  std::map<std::string, std::unique_ptr<CdrChain>> chains_;
  rclcpp::Node::SharedPtr source_, output_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> publishers_;
};

void counts(std::ostream &out, const std::map<std::string, size_t> &values) {
  out << "{";
  bool first = true;
  for (const auto &[key, value] : values) {
    if (!first) out << ",";
    first = false;
    out << json_quote(key) << ":" << value;
  }
  out << "}";
}

int run(const Arguments &args) {
  // Reserve before context initialization. The stdlib public launcher also
  // covers pre-main shared-library exits and preserves previous attempts.
  std::filesystem::create_directories(args.output.parent_path());
  const int receipt_fd = ::open(args.output.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (receipt_fd < 0) {
    if (errno == EEXIST) throw std::runtime_error("receipt already exists; use a new path and preserve the previous attempt");
    throw std::runtime_error("cannot reserve new shadow receipt");
  }
  const std::string initial = "{\"status\":\"INCOMPLETE\",\"termination\":\"not_finalized\",\"scope\":\"Attempt reserved before ROS init; never a qualification pass.\"}\n";
  const auto written = ::write(receipt_fd, initial.data(), initial.size());
  const int close_result = ::close(receipt_fd);
  if (written != static_cast<ssize_t>(initial.size()) || close_result != 0)
    throw std::runtime_error("cannot initialize reserved shadow receipt");
  const auto previous_int = std::signal(SIGINT, signal_handler);
  const auto previous_term = std::signal(SIGTERM, signal_handler);
  const auto started = std::chrono::steady_clock::now();
  std::unique_ptr<SensorIngress> ingress;
  std::string status = "COMPLETED", termination = "duration", error, cleanup_error;
  std::map<std::string, size_t> forwarded, denials;
  std::map<std::string, std::string> allowlist, input_chains;
  auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  };
  try {
    ingress = std::make_unique<SensorIngress>(args);
    while (!stop_signal && elapsed() < args.duration_s) {
      if (elapsed() < args.discovery_delay_s) {
        // Optional finite discovery pause used by the queue-order qualification.
        // DDS continues receiving; the volatile depth-one queue is never replayed.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } else ingress->spin_once();
    }
    if (stop_signal) {
      status = "INTERRUPTED";
      termination = stop_signal == SIGINT ? "SIGINT" : "SIGTERM";
    }
  } catch (const std::exception &failure) {
    status = "FAILED"; termination = "exception"; error = failure.what();
  }
  if (ingress) {
    forwarded = ingress->forwarded; denials = ingress->denials; allowlist = ingress->allowlist;
    input_chains = ingress->payload_chains();
    try { ingress->close(); }
    catch (const std::exception &failure) { status = "FAILED"; cleanup_error = failure.what(); }
  }
  std::signal(SIGINT, previous_int);
  std::signal(SIGTERM, previous_term);
  std::ostringstream result;
  result << std::setprecision(17)
    << "{\"status\":" << json_quote(status)
    << ",\"termination\":" << json_quote(termination)
    << ",\"error\":" << (error.empty() ? "null" : json_quote(error))
    << ",\"cleanup_error\":" << (cleanup_error.empty() ? "null" : json_quote(cleanup_error))
    << ",\"source_domain\":" << args.source_domain
    << ",\"research_domain\":" << args.research_domain
    << ",\"wall_s\":" << elapsed()
    << ",\"discovery_delay_s\":" << args.discovery_delay_s
    << ",\"input_observed\":" << (forwarded.empty() ? "false" : "true")
    << ",\"forwarded\":";
  counts(result, forwarded);
  result << ",\"denials\":";
  counts(result, denials);
  result << ",\"audit_payloads\":" << (args.audit_payloads ? "true" : "false")
    << ",\"input_cdr_chains\":{";
  bool chain_first = true;
  for (const auto &[topic, digest] : input_chains) {
    if (!chain_first) result << ",";
    chain_first = false;
    result << json_quote(topic) << ":" << json_quote(digest);
  }
  result << "},\"cdr_chain_encoding\":\"sha256(uint64_le_length || received_cdr_bytes, per topic in forwarded order)\",\"allowlist\":[";
  bool first = true;
  for (const auto &[topic, wire] : allowlist) {
    if (!first) result << ",";
    first = false;
    result << "{\"topic\":" << json_quote(topic) << ",\"type\":" << json_quote(wire) << "}";
  }
  result << "],\"scope\":\"Finite one-way raw CDR sensor ingress, per-sample discovered writer GID check, zero source-domain writers. No cache/restamp/TF/cmd/authority/consent/clock. Completion is a process result; target shadow, identity authentication, transport security and physical acceptance remain unqualified.\"}\n";
  std::filesystem::create_directories(args.output.parent_path());
  std::ofstream file(args.output);
  file << result.str();
  file.flush();
  if (!file) throw std::runtime_error("cannot persist shadow receipt");
  std::cout << result.str();
  if (status == "FAILED") return 1;
  return stop_signal ? 128 + stop_signal : 0;
}
}  // namespace

int main(int argc, char **argv) {
  try { return run(arguments(argc, argv)); }
  catch (const std::exception &failure) {
    std::cerr << "shadow ingress: " << failure.what() << "\n";
    return 2;
  }
}
