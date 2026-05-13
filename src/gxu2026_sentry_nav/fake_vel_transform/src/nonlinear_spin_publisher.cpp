#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <random>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/rmuc_robot_control.hpp"

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

template <typename T>
T clamp(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

}  // namespace

namespace fake_vel_transform
{

class NonlinearSpinPublisher : public rclcpp::Node
{
public:
  explicit NonlinearSpinPublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("nonlinear_spin_publisher", options)
  {
    this->declare_parameter<bool>("enabled", true);
    this->declare_parameter<bool>("start_on_first_trigger", false);
    this->declare_parameter<std::string>("start_trigger_topic", "controller_server/FollowPath/local_plan");
    this->declare_parameter<std::string>(
      "start_trigger_msg_type", "nav_msgs/Path");
    this->declare_parameter<bool>("trigger_require_nonzero", false);
    this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    // 变速小陀螺：中心速度 + 变频变幅正弦叠加。
    this->declare_parameter<double>("center_speed", 6.28);
    this->declare_parameter<double>("range_speed", 2.0);
    this->declare_parameter<double>("primary_amp_ratio", 0.65);
    this->declare_parameter<double>("secondary_amp_ratio", 0.35);
    this->declare_parameter<double>("primary_freq_hz", 0.8);
    this->declare_parameter<double>("secondary_freq_hz", 1.7);
    this->declare_parameter<double>("harmonic_phase_offset", 1.57);
    this->declare_parameter<double>("amp_scale_min", 0.7);
    this->declare_parameter<double>("amp_scale_max", 1.3);
    this->declare_parameter<double>("freq_scale_min", 0.75);
    this->declare_parameter<double>("freq_scale_max", 1.25);
    this->declare_parameter<double>("modulation_slew_rate", 2.5);
    this->declare_parameter<double>("max_abs_speed", 10.0);
    this->declare_parameter<double>("min_abs_speed", 0.0);
    this->declare_parameter<double>("target_update_period", 0.15);
    this->declare_parameter<double>("accel_limit", 30.0);
    this->declare_parameter<int64_t>("seed", 1);
    this->declare_parameter<double>("start_delay_sec", 0.5);
    this->declare_parameter<bool>("stop_on_idle", true);
    this->declare_parameter<double>("idle_timeout_sec", 0.5);
    this->declare_parameter<std::string>("robot_control_topic", "robot_control");

    this->get_parameter("enabled", enabled_);
    this->get_parameter("start_on_first_trigger", start_on_first_trigger_);
    this->get_parameter("start_trigger_topic", start_trigger_topic_);
    this->get_parameter("start_trigger_msg_type", start_trigger_msg_type_);
    this->get_parameter("trigger_require_nonzero", trigger_require_nonzero_);
    this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("center_speed", center_speed_);
    this->get_parameter("range_speed", range_speed_);
    this->get_parameter("primary_amp_ratio", primary_amp_ratio_);
    this->get_parameter("secondary_amp_ratio", secondary_amp_ratio_);
    this->get_parameter("primary_freq_hz", primary_freq_hz_);
    this->get_parameter("secondary_freq_hz", secondary_freq_hz_);
    this->get_parameter("harmonic_phase_offset", harmonic_phase_offset_);
    this->get_parameter("amp_scale_min", amp_scale_min_);
    this->get_parameter("amp_scale_max", amp_scale_max_);
    this->get_parameter("freq_scale_min", freq_scale_min_);
    this->get_parameter("freq_scale_max", freq_scale_max_);
    this->get_parameter("modulation_slew_rate", modulation_slew_rate_);
    this->get_parameter("max_abs_speed", max_abs_speed_);
    this->get_parameter("min_abs_speed", min_abs_speed_);
    this->get_parameter("target_update_period", target_update_period_);
    this->get_parameter("accel_limit", accel_limit_);
    this->get_parameter("seed", seed_);
    this->get_parameter("start_delay_sec", start_delay_sec_);
    this->get_parameter("stop_on_idle", stop_on_idle_);
    this->get_parameter("idle_timeout_sec", idle_timeout_sec_);
    this->get_parameter("robot_control_topic", robot_control_topic_);

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 50.0;
    }

    if (target_update_period_ <= 0.0) {
      target_update_period_ = 0.15;
    }

    if (primary_freq_hz_ <= 0.0) {
      primary_freq_hz_ = 0.8;
    }

    if (secondary_freq_hz_ <= 0.0) {
      secondary_freq_hz_ = 1.7;
    }

    if (amp_scale_min_ < 0.0) {
      amp_scale_min_ = 0.0;
    }

    if (amp_scale_max_ < amp_scale_min_) {
      std::swap(amp_scale_min_, amp_scale_max_);
    }

    if (freq_scale_min_ <= 0.0) {
      freq_scale_min_ = 0.1;
    }

    if (freq_scale_max_ < freq_scale_min_) {
      std::swap(freq_scale_min_, freq_scale_max_);
    }

    if (modulation_slew_rate_ <= 0.0) {
      modulation_slew_rate_ = 2.5;
    }

    primary_amp_ratio_ = std::max(0.0, primary_amp_ratio_);
    secondary_amp_ratio_ = std::max(0.0, secondary_amp_ratio_);
    const double amp_sum = primary_amp_ratio_ + secondary_amp_ratio_;
    if (amp_sum <= 1e-6) {
      primary_amp_ratio_ = 1.0;
      secondary_amp_ratio_ = 0.0;
    } else {
      primary_amp_ratio_ /= amp_sum;
      secondary_amp_ratio_ /= amp_sum;
    }

    if (max_abs_speed_ < 0.0) {
      max_abs_speed_ = std::abs(max_abs_speed_);
    }

    if (min_abs_speed_ < 0.0) {
      min_abs_speed_ = 0.0;
    }

    if (min_abs_speed_ > max_abs_speed_) {
      std::swap(min_abs_speed_, max_abs_speed_);
    }

    if (seed_ == 0) {
      std::random_device rd;
      rng_.seed(static_cast<std::mt19937::result_type>(rd()));
    } else {
      rng_.seed(static_cast<std::mt19937::result_type>(seed_));
    }

    dist_unit_ = std::uniform_real_distribution<double>(0.0, 1.0);

    cmd_spin_pub_ = this->create_publisher<example_interfaces::msg::Float32>(cmd_spin_topic_, 1);

    if (start_on_first_trigger_) {
      const std::string type = start_trigger_msg_type_;
      const std::string type_lower = toLower(type);
      if (type_lower == "nav_msgs/path" || type_lower == "path") {
        trigger_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
          start_trigger_topic_, rclcpp::QoS(10),
          std::bind(&NonlinearSpinPublisher::onTriggerPath, this, std::placeholders::_1));
      } else if (type_lower == "geometry_msgs/twist" || type_lower == "twist") {
        trigger_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          start_trigger_topic_, rclcpp::QoS(10),
          std::bind(&NonlinearSpinPublisher::onTriggerTwist, this, std::placeholders::_1));
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Unknown start_trigger_msg_type='%s', fallback to nav_msgs/Path",
          start_trigger_msg_type_.c_str());
        trigger_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
          start_trigger_topic_, rclcpp::QoS(10),
          std::bind(&NonlinearSpinPublisher::onTriggerPath, this, std::placeholders::_1));
      }
    }

    // Subscribe to robot_control (RMUCRobotControl msg) for chassis_spin master toggle
    robot_control_sub_ = this->create_subscription<sp_msgs::msg::RMUCRobotControl>(
      robot_control_topic_, rclcpp::QoS(10),
      std::bind(&NonlinearSpinPublisher::onRobotControl, this, std::placeholders::_1));

    // Init state
    amp_scale_ = clamp(1.0, amp_scale_min_, amp_scale_max_);
    freq_scale_ = clamp(1.0, freq_scale_min_, freq_scale_max_);
    amp_scale_target_ = amp_scale_;
    freq_scale_target_ = freq_scale_;

    phase_primary_ = kTwoPi * dist_unit_(rng_);
    phase_secondary_ = kTwoPi * dist_unit_(rng_);

    w_current_ = clamp(center_speed_, -max_abs_speed_, max_abs_speed_);
    w_target_ = w_current_;

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    // Use sim-time-aware timer instead of wall_timer to keep dt calculations
    // consistent with simulation clock and avoid time jump issues.
    timer_ = rclcpp::create_timer(
      this, this->get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NonlinearSpinPublisher::onTimer, this));

    start_time_ = this->get_clock()->now();
    last_time_ = start_time_;
    last_target_update_time_ = start_time_;
    retargetModulation();

    RCLCPP_INFO(
      this->get_logger(),
      "NonlinearSpinPublisher: start_on_first_trigger=%s trigger_topic=%s trigger_type=%s cmd_spin_topic=%s center=%.3f range=%.3f freq=(%.3f,%.3f) amp_ratio=(%.2f,%.2f) amp_scale=[%.2f,%.2f] freq_scale=[%.2f,%.2f] max_abs=%.3f update=%.3fs accel=%.3f publish=%.1fHz seed=%ld",
      start_on_first_trigger_ ? "true" : "false",
      start_trigger_topic_.c_str(),
      start_trigger_msg_type_.c_str(),
      cmd_spin_topic_.c_str(),
      center_speed_,
      range_speed_,
      primary_freq_hz_,
      secondary_freq_hz_,
      primary_amp_ratio_,
      secondary_amp_ratio_,
      amp_scale_min_,
      amp_scale_max_,
      freq_scale_min_,
      freq_scale_max_,
      max_abs_speed_,
      target_update_period_,
      accel_limit_,
      publish_rate_hz_,
      static_cast<long>(seed_));
  }

private:
  static std::string toLower(const std::string & s)
  {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
  }

  void onTriggerPath(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
  {
    last_msg_time_ = this->get_clock()->now();
    if (!triggered_) {
      triggered_ = true;
      RCLCPP_INFO(this->get_logger(), "NonlinearSpinPublisher triggered: start spinning now.");
    }
  }

  void onTriggerTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    last_msg_time_ = this->get_clock()->now();
    if (triggered_) {
      return;
    }

    if (trigger_require_nonzero_) {
      const bool nonzero =
        std::abs(msg->linear.x) > 1e-6 || std::abs(msg->linear.y) > 1e-6 ||
        std::abs(msg->angular.z) > 1e-6;
      if (!nonzero) {
        return;
      }
    }

    triggered_ = true;
    RCLCPP_INFO(this->get_logger(), "NonlinearSpinPublisher triggered: start spinning now.");
  }

  void onRobotControl(const sp_msgs::msg::RMUCRobotControl::ConstSharedPtr msg)
  {
    spin_enabled_ = msg->chassis_spin;
  }

  void onTimer()
  {
    const auto now = this->get_clock()->now();

    // Static config toggle: disabled via parameter, publish zero once and return.
    if (!enabled_) {
      if (w_current_ != 0.0) {
        w_current_ = 0.0;
        w_target_ = 0.0;
        example_interfaces::msg::Float32 zero_msg;
        zero_msg.data = 0.0f;
        cmd_spin_pub_->publish(zero_msg);
      }
      last_time_ = now;
      return;
    }

    // chassis_spin master toggle from BT RobotControl
    if (!spin_enabled_) {
      if (w_current_ != 0.0) {
        w_current_ = 0.0;
        w_target_ = 0.0;
        example_interfaces::msg::Float32 zero_msg;
        zero_msg.data = 0.0f;
        cmd_spin_pub_->publish(zero_msg);
      }
      last_time_ = now;
      return;
    }

    if (start_on_first_trigger_ && !triggered_) {
      return;
    }

    // idle 超时停止自旋：当 trigger topic 超过 idle_timeout_sec 没有新消息，停止自旋并发布零速
    if (triggered_ && stop_on_idle_ && start_on_first_trigger_) {
      const double idle = (now - last_msg_time_).seconds();
      if (idle > idle_timeout_sec_) {
        triggered_ = false;
        w_current_ = 0.0;
        w_target_ = 0.0;
        example_interfaces::msg::Float32 zero_msg;
        zero_msg.data = 0.0f;
        cmd_spin_pub_->publish(zero_msg);
        RCLCPP_INFO(
          this->get_logger(),
          "NonlinearSpinPublisher: idle %.2fs > timeout %.2fs, stopping spin.",
          idle, idle_timeout_sec_);
        last_time_ = now;
        return;
      }
    }

    if ((now - start_time_).seconds() < start_delay_sec_) {
      return;
    }

    double dt = (now - last_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      last_time_ = now;
      return;
    }

    // Cap dt to avoid huge jumps if the process was paused.
    dt = std::min(dt, 0.1);

    if ((now - last_target_update_time_).seconds() >= target_update_period_) {
      retargetModulation();
      last_target_update_time_ = now;
    }

    if (modulation_slew_rate_ > 0.0 && std::isfinite(modulation_slew_rate_)) {
      const double max_scale_step = modulation_slew_rate_ * dt;
      amp_scale_ += clamp(amp_scale_target_ - amp_scale_, -max_scale_step, max_scale_step);
      freq_scale_ += clamp(freq_scale_target_ - freq_scale_, -max_scale_step, max_scale_step);
    } else {
      amp_scale_ = amp_scale_target_;
      freq_scale_ = freq_scale_target_;
    }

    amp_scale_ = clamp(amp_scale_, amp_scale_min_, amp_scale_max_);
    freq_scale_ = clamp(freq_scale_, freq_scale_min_, freq_scale_max_);

    w_target_ = evaluateSpinTarget(dt);

    // Acceleration limiting (rad/s^2) to keep it physically achievable.
    if (accel_limit_ > 0.0 && std::isfinite(accel_limit_)) {
      const double max_delta = accel_limit_ * dt;
      const double delta = clamp(w_target_ - w_current_, -max_delta, max_delta);
      w_current_ += delta;
    } else {
      w_current_ = w_target_;
    }

    w_current_ = clamp(w_current_, -max_abs_speed_, max_abs_speed_);

    if (min_abs_speed_ > 0.0 && std::abs(w_current_) < min_abs_speed_) {
      const double sign_hint = (std::abs(w_target_) > 1e-6) ? w_target_ : w_current_;
      w_current_ = (sign_hint >= 0.0) ? min_abs_speed_ : -min_abs_speed_;
    }

    example_interfaces::msg::Float32 msg;
    msg.data = static_cast<float>(w_current_);
    cmd_spin_pub_->publish(msg);

    last_time_ = now;
  }

  void retargetModulation()
  {
    amp_scale_target_ =
      amp_scale_min_ + (amp_scale_max_ - amp_scale_min_) * dist_unit_(rng_);
    freq_scale_target_ =
      freq_scale_min_ + (freq_scale_max_ - freq_scale_min_) * dist_unit_(rng_);
  }

  double evaluateSpinTarget(double dt)
  {
    const double freq_primary = std::max(0.05, primary_freq_hz_ * freq_scale_);
    const double freq_secondary = std::max(0.05, secondary_freq_hz_ * freq_scale_);

    phase_primary_ = std::fmod(phase_primary_ + kTwoPi * freq_primary * dt, kTwoPi);
    phase_secondary_ = std::fmod(phase_secondary_ + kTwoPi * freq_secondary * dt, kTwoPi);

    const double total_amp = std::max(0.0, range_speed_) * amp_scale_;
    const double amp_primary = total_amp * primary_amp_ratio_;
    const double amp_secondary = total_amp * secondary_amp_ratio_;

    const double target =
      center_speed_ +
      amp_primary * std::sin(phase_primary_) +
      amp_secondary * std::sin(phase_secondary_ + harmonic_phase_offset_);
    return clamp(target, -max_abs_speed_, max_abs_speed_);
  }

private:
  std::string cmd_spin_topic_;
  std::string start_trigger_topic_;
  std::string start_trigger_msg_type_;
  std::string robot_control_topic_;

  bool enabled_{true};
  bool start_on_first_trigger_{false};
  bool triggered_{false};
  bool trigger_require_nonzero_{false};
  bool stop_on_idle_{true};
  bool spin_enabled_{true};

  double publish_rate_hz_{50.0};
  double center_speed_{6.28};
  double range_speed_{2.0};
  double primary_amp_ratio_{0.65};
  double secondary_amp_ratio_{0.35};
  double primary_freq_hz_{0.8};
  double secondary_freq_hz_{1.7};
  double harmonic_phase_offset_{1.57};
  double amp_scale_min_{0.7};
  double amp_scale_max_{1.3};
  double freq_scale_min_{0.75};
  double freq_scale_max_{1.25};
  double modulation_slew_rate_{2.5};
  double max_abs_speed_{10.0};
  double min_abs_speed_{0.0};
  double target_update_period_{0.15};
  double accel_limit_{30.0};
  int64_t seed_{1};
  double start_delay_sec_{0.5};
  double idle_timeout_sec_{0.5};

  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr cmd_spin_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trigger_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr trigger_twist_sub_;
  rclcpp::Subscription<sp_msgs::msg::RMUCRobotControl>::SharedPtr robot_control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time last_time_;
  rclcpp::Time last_target_update_time_;
  rclcpp::Time last_msg_time_;

  double w_current_{0.0};
  double w_target_{0.0};
  double amp_scale_{1.0};
  double freq_scale_{1.0};
  double amp_scale_target_{1.0};
  double freq_scale_target_{1.0};
  double phase_primary_{0.0};
  double phase_secondary_{0.0};

  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_unit_;
};

}  // namespace fake_vel_transform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fake_vel_transform::NonlinearSpinPublisher>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
