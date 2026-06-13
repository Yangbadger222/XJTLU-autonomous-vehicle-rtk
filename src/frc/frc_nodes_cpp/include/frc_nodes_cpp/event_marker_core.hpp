#pragma once

#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace frc_nodes_cpp
{

constexpr uint8_t kCtrlModeHost = 0;
constexpr uint8_t kCtrlModeGamepad = 1;
constexpr uint8_t kCtrlModeDisabled = 2;
constexpr uint8_t kPsbSelect = 1;

struct EventMarkerConfig
{
  double stuck_odom_speed{0.05};
  double stuck_cmd_speed{0.2};
  double stuck_duration_s{3.0};
  double nearcol_clearance_m{0.35};
  double nearcol_duration_s{0.5};
  int jerk_min_samples{200};
  double cooldown_s{8.0};
};

struct EventDecision
{
  std::string type;
  std::string severity;
  std::string note;
};

class RunningStats
{
public:
  void push(const double x)
  {
    ++n_;
    const double delta = x - mean_;
    mean_ += delta / static_cast<double>(n_);
    m2_ += delta * (x - mean_);
  }

  bool exceeds3Sigma(const double x, const int min_samples) const
  {
    return n_ >= min_samples && stddev() > 1e-9 &&
           x > mean_ + 3.0 * stddev();
  }

private:
  double stddev() const
  {
    return n_ > 1 ? std::sqrt(m2_ / static_cast<double>(n_)) : 0.0;
  }

  int n_{0};
  double mean_{0.0};
  double m2_{0.0};
};

class EventMarkerCore
{
public:
  explicit EventMarkerCore(EventMarkerConfig config)
  : config_(config)
  {
  }

  std::vector<EventDecision> onChassis(
    const uint8_t ctrl_mode, const uint8_t ps2_key, const double now_s)
  {
    std::vector<EventDecision> events;

    if (last_ctrl_mode_.has_value() && last_ctrl_mode_.value() == kCtrlModeHost &&
      (ctrl_mode == kCtrlModeGamepad || ctrl_mode == kCtrlModeDisabled))
    {
      auto ev = emitIfAllowed(
        "takeover", "gold",
        "ctrl_mode " + std::to_string(last_ctrl_mode_.value()) + "->" +
        std::to_string(ctrl_mode),
        now_s, false);
      if (ev.has_value()) {
        events.push_back(ev.value());
      }
    }
    last_ctrl_mode_ = ctrl_mode;

    if (ps2_key == kPsbSelect && last_ps2_key_ != kPsbSelect) {
      auto ev = emitIfAllowed("manual", "gold", "ps2 SELECT", now_s, true);
      if (ev.has_value()) {
        events.push_back(ev.value());
      }
    }
    last_ps2_key_ = ps2_key;

    return events;
  }

  std::optional<EventDecision> onCmd(
    const double vx, const double wz, const double now_s)
  {
    cmd_vx_ = vx;
    cmd_wz_ = wz;

    if (last_cmd_.has_value()) {
      const double dt = now_s - last_cmd_->first;
      if (dt > 1e-3 && dt < 1.0) {
        const double accel = (vx - last_cmd_->second) / dt;
        if (last_accel_.has_value()) {
          const double dt2 = now_s - last_accel_->first;
          if (dt2 > 1e-3 && dt2 < 1.0) {
            const double jerk = std::abs((accel - last_accel_->second) / dt2);
            if (jerk_stats_.exceeds3Sigma(jerk, config_.jerk_min_samples)) {
              auto ev = emitIfAllowed(
                "jerk", "bronze", "jerk=" + formatFixed(jerk, 1), now_s, false);
              jerk_stats_.push(jerk);
              last_accel_ = std::make_pair(now_s, accel);
              last_cmd_ = std::make_pair(now_s, vx);
              return ev;
            }
            jerk_stats_.push(jerk);
          }
        }
        last_accel_ = std::make_pair(now_s, accel);
      }
    }

    last_cmd_ = std::make_pair(now_s, vx);
    return std::nullopt;
  }

  void onOdomVelocity(const double vx, const double vy)
  {
    odom_speed_ = std::hypot(vx, vy);
  }

  std::optional<EventDecision> onTick(const double now_s)
  {
    if (odom_speed_ < config_.stuck_odom_speed &&
      std::abs(cmd_vx_) > config_.stuck_cmd_speed)
    {
      if (!stuck_since_.has_value()) {
        stuck_since_ = now_s;
      } else if (now_s - stuck_since_.value() >= config_.stuck_duration_s) {
        stuck_since_.reset();
        return emitIfAllowed(
          "stuck", "silver",
          "v=" + formatFixed(odom_speed_, 3) + " cmd=" + formatFixed(cmd_vx_, 2),
          now_s, false);
      }
    } else {
      stuck_since_.reset();
    }
    return std::nullopt;
  }

  std::optional<EventDecision> onCostmapClearance(
    const std::optional<double> clearance_m, const double now_s)
  {
    if (!clearance_m.has_value()) {
      nearcol_since_.reset();
      return std::nullopt;
    }

    if (clearance_m.value() < config_.nearcol_clearance_m) {
      if (!nearcol_since_.has_value()) {
        nearcol_since_ = now_s;
      } else if (now_s - nearcol_since_.value() >= config_.nearcol_duration_s) {
        nearcol_since_.reset();
        return emitIfAllowed(
          "near_collision", "silver",
          "clearance=" + formatFixed(clearance_m.value(), 2) + "m",
          now_s, false);
      }
    } else {
      nearcol_since_.reset();
    }
    return std::nullopt;
  }

  std::optional<EventDecision> onRecovery(
    const std::string & node_name, const std::string & status, const double now_s)
  {
    static const std::set<std::string> kRecoveryNodes{
      "Spin", "BackUp", "Wait", "DriveOnHeading"};
    if (status == "RUNNING" && kRecoveryNodes.count(node_name) > 0) {
      return emitIfAllowed("recovery", "silver", node_name, now_s, false);
    }
    return std::nullopt;
  }

  std::optional<EventDecision> onPlanRms(const double rms_m, const double now_s)
  {
    if (plan_stats_.exceeds3Sigma(rms_m, 30)) {
      auto ev = emitIfAllowed(
        "plan_instability", "bronze",
        "rms=" + formatFixed(rms_m, 2) + "m", now_s, false);
      plan_stats_.push(rms_m);
      return ev;
    }
    plan_stats_.push(rms_m);
    return std::nullopt;
  }

private:
  std::optional<EventDecision> emitIfAllowed(
    const std::string & type,
    const std::string & severity,
    const std::string & note,
    const double now_s,
    const bool bypass_cooldown)
  {
    const auto it = last_emit_.find(type);
    if (!bypass_cooldown && it != last_emit_.end() &&
      now_s - it->second < config_.cooldown_s)
    {
      return std::nullopt;
    }
    last_emit_[type] = now_s;
    return EventDecision{type, severity, note};
  }

  static std::string formatFixed(const double value, const int precision)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
  }

  EventMarkerConfig config_;
  std::optional<uint8_t> last_ctrl_mode_;
  uint8_t last_ps2_key_{0};
  double odom_speed_{0.0};
  double cmd_vx_{0.0};
  double cmd_wz_{0.0};
  std::optional<double> stuck_since_;
  std::optional<double> nearcol_since_;
  std::optional<std::pair<double, double>> last_cmd_;
  std::optional<std::pair<double, double>> last_accel_;
  RunningStats jerk_stats_;
  RunningStats plan_stats_;
  std::map<std::string, double> last_emit_;
};

}  // namespace frc_nodes_cpp
