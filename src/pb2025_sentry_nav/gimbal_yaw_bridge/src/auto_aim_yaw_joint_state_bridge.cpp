#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "gimbal_yaw_interfaces/msg/float32_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace pb_gimbal_yaw_bridge
{

constexpr double kPi = 3.14159265358979323846;

class AutoAimYawJointStateBridge final : public rclcpp::Node
{
public:
  explicit AutoAimYawJointStateBridge(const rclcpp::NodeOptions & options)
  : Node("auto_aim_yaw_joint_state_bridge", options)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "auto_aim_yaw");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "gimbal_joint_state");
    joint_name_ = this->declare_parameter<std::string>("joint_name", "gimbal_yaw_joint");

    input_is_degrees_ = this->declare_parameter<bool>("input_is_degrees", false);
    invert_sign_ = this->declare_parameter<bool>("invert_sign", false);
    scale_ = this->declare_parameter<double>("scale", 1.0);
    offset_ = this->declare_parameter<double>("offset", 0.0);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 0.0);

    default_yaw_rad_ = this->declare_parameter<double>("default_yaw_rad", 0.0);
    publish_on_startup_ = this->declare_parameter<bool>("publish_on_startup", true);
    stamp_offset_sec_ = this->declare_parameter<double>("stamp_offset_sec", 0.0);
    use_stamped_msg_ = this->declare_parameter<bool>("use_stamped_msg", false);

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::QoS(10));

    if (use_stamped_msg_) {
      yaw_stamped_sub_ = this->create_subscription<gimbal_yaw_interfaces::msg::Float32Stamped>(
        input_topic_, rclcpp::QoS(10),
        std::bind(
          &AutoAimYawJointStateBridge::onYawStamped, this, std::placeholders::_1));
    } else {
      yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        input_topic_, rclcpp::QoS(10),
        std::bind(&AutoAimYawJointStateBridge::onYaw, this, std::placeholders::_1));
    }

    if (publish_on_startup_) {
      last_yaw_rad_ = default_yaw_rad_;
      have_last_ = true;
      last_stamp_ = this->get_clock()->now();
      publish(last_yaw_rad_, last_stamp_);
    }

    if (publish_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&AutoAimYawJointStateBridge::publishLast, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Bridging '%s' (std_msgs/Float32) -> '%s' (sensor_msgs/JointState), joint_name='%s'",
      input_topic_.c_str(), output_topic_.c_str(), joint_name_.c_str());
  }

private:
  void onYaw(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_yaw_rad_ = convertToRad(static_cast<double>(msg->data));
    have_last_ = true;
    last_stamp_ = this->get_clock()->now();
    publish(last_yaw_rad_, last_stamp_);
  }

  void onYawStamped(const gimbal_yaw_interfaces::msg::Float32Stamped::SharedPtr msg)
  {
    last_yaw_rad_ = convertToRad(static_cast<double>(msg->data));
    have_last_ = true;
    last_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    publish(last_yaw_rad_, last_stamp_);
  }

  double convertToRad(double raw) const
  {
    double value = raw;
    if (input_is_degrees_) {
      value = value * kPi / 180.0;
    }
    value = value * scale_ + offset_;
    if (invert_sign_) {
      value = -value;
    }
    return value;
  }

  void publishLast()
  {
    if (!have_last_) {
      return;
    }
    publish(last_yaw_rad_, last_stamp_);
  }

  void publish(double yaw_rad, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = (stamp + rclcpp::Duration::from_seconds(stamp_offset_sec_));
    js.name = {joint_name_};
    js.position = {yaw_rad};
    joint_pub_->publish(js);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string joint_name_;

  bool input_is_degrees_{false};
  bool invert_sign_{false};
  double scale_{1.0};
  double offset_{0.0};
  double publish_rate_hz_{0.0};
  double default_yaw_rad_{0.0};
  bool publish_on_startup_{true};
  double stamp_offset_sec_{0.0};
  bool use_stamped_msg_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<gimbal_yaw_interfaces::msg::Float32Stamped>::SharedPtr yaw_stamped_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_last_{false};
  double last_yaw_rad_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace pb_gimbal_yaw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pb_gimbal_yaw_bridge::AutoAimYawJointStateBridge>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
