#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <random>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "sp_msgs/msg/rmul.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

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
    this->declare_parameter<bool>("enabled", false);
    this->declare_parameter<bool>("start_on_first_trigger", false);
    this->declare_parameter<std::string>("start_trigger_topic", "controller_server/FollowPath/local_plan");
    this->declare_parameter<std::string>(
      "start_trigger_msg_type", "nav_msgs/Path");
    this->declare_parameter<bool>("trigger_require_nonzero", false);
    this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    // Nonlinear spin profile: band-limited random target with acceleration limiting.
    this->declare_parameter<double>("center_speed", 6.28);
    this->declare_parameter<double>("range_speed", 2.0);
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

    dist_unit_ = std::uniform_real_distribution<double>(-1.0, 1.0);

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

    // Subscribe to robot_control (RMUL msg) for chassis_spin master toggle
    robot_control_sub_ = this->create_subscription<sp_msgs::msg::RMUL>(
      robot_control_topic_, rclcpp::QoS(10),
      std::bind(&NonlinearSpinPublisher::onRobotControl, this, std::placeholders::_1));

    // Init state
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
    last_msg_time_ = start_time_;

    RCLCPP_INFO(
      this->get_logger(),
      "NonlinearSpinPublisher: enabled=%s start_on_first_trigger=%s trigger_topic=%s trigger_type=%s cmd_spin_topic=%s robot_control_topic=%s center=%.3f range=%.3f max_abs=%.3f update=%.3fs accel=%.3f publish=%.1fHz seed=%ld stop_on_idle=%s idle_timeout=%.2fs",
      enabled_ ? "true" : "false",
      start_on_first_trigger_ ? "true" : "false",
      start_trigger_topic_.c_str(),
      start_trigger_msg_type_.c_str(),
      cmd_spin_topic_.c_str(), robot_control_topic_.c_str(),
      center_speed_, range_speed_, max_abs_speed_, target_update_period_,
      accel_limit_, publish_rate_hz_, static_cast<long>(seed_),
      stop_on_idle_ ? "true" : "false", idle_timeout_sec_);
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

  void onRobotControl(const sp_msgs::msg::RMUL::ConstSharedPtr msg)
  {
    chassis_spin_enabled_ = msg->chassis_spin;
  }

  void onTimer()
  {
    const auto now = this->get_clock()->now();

    if (!enabled_) {
      return;
    }

    // chassis_spin master toggle from BT RobotControl
    if (!chassis_spin_enabled_) {
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
      retarget();
      last_target_update_time_ = now;
    }

    // Acceleration limiting (rad/s^2) to keep it physically achievable.
    if (accel_limit_ > 0.0 && std::isfinite(accel_limit_)) {
      const double max_delta = accel_limit_ * dt;
      const double delta = clamp(w_target_ - w_current_, -max_delta, max_delta);
      w_current_ += delta;
    } else {
      w_current_ = w_target_;
    }

    w_current_ = clamp(w_current_, -max_abs_speed_, max_abs_speed_);

    example_interfaces::msg::Float32 msg;
    msg.data = static_cast<float>(w_current_);
    cmd_spin_pub_->publish(msg);

    last_time_ = now;
  }

  void retarget()
  {
    // New target: center + range * u, where u ~ U[-1,1].
    double candidate = center_speed_ + range_speed_ * dist_unit_(rng_);

    // Enforce min_abs_speed (avoid sitting near 0 unless explicitly allowed).
    if (min_abs_speed_ > 0.0 && std::abs(candidate) < min_abs_speed_) {
      candidate = (candidate >= 0.0 ? min_abs_speed_ : -min_abs_speed_);
    }

    candidate = clamp(candidate, -max_abs_speed_, max_abs_speed_);

    // If range is 0, still allow a tiny dither to avoid perfect const speed.
    if (range_speed_ == 0.0) {
      candidate = clamp(candidate + 0.05 * dist_unit_(rng_), -max_abs_speed_, max_abs_speed_);
    }

    w_target_ = candidate;
  }

private:
  std::string cmd_spin_topic_;
  std::string start_trigger_topic_;
  std::string start_trigger_msg_type_;

  bool enabled_{false};
  bool start_on_first_trigger_{false};
  bool triggered_{false};
  bool trigger_require_nonzero_{false};

  double publish_rate_hz_{50.0};
  double center_speed_{6.28};
  double range_speed_{2.0};
  double max_abs_speed_{10.0};
  double min_abs_speed_{0.0};
  double target_update_period_{0.15};
  double accel_limit_{30.0};
  int64_t seed_{1};
  double start_delay_sec_{0.5};

  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr cmd_spin_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trigger_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr trigger_twist_sub_;
  rclcpp::Subscription<sp_msgs::msg::RMUL>::SharedPtr robot_control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time last_time_;
  rclcpp::Time last_target_update_time_;

  double w_current_{0.0};
  double w_target_{0.0};

  bool stop_on_idle_{true};
  double idle_timeout_sec_{0.5};
  rclcpp::Time last_msg_time_;

  std::string robot_control_topic_;
  bool chassis_spin_enabled_{false};

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
