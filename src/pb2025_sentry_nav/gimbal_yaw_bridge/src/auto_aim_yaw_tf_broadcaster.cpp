#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sp_msgs/msg/float32_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace gimbal_yaw_bridge
{

constexpr double kPi = 3.14159265358979323846;

class AutoAimYawTfBroadcaster final : public rclcpp::Node
{
public:
  explicit AutoAimYawTfBroadcaster(const rclcpp::NodeOptions & options)
  : Node("auto_aim_yaw_tf_broadcaster", options)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "auto_aim_yaw");
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "chassis");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "gimbal_yaw_auto");

    translation_x_ = this->declare_parameter<double>("translation_x", 0.0);
    translation_y_ = this->declare_parameter<double>("translation_y", 0.0);
    translation_z_ = this->declare_parameter<double>("translation_z", 0.0);

    input_is_degrees_ = this->declare_parameter<bool>("input_is_degrees", false);
    invert_sign_ = this->declare_parameter<bool>("invert_sign", false);
    scale_ = this->declare_parameter<double>("scale", 1.0);
    offset_ = this->declare_parameter<double>("offset", 0.0);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 0.0);

    default_yaw_rad_ = this->declare_parameter<double>("default_yaw_rad", 0.0);
    publish_on_startup_ = this->declare_parameter<bool>("publish_on_startup", true);
    stamp_offset_sec_ = this->declare_parameter<double>("stamp_offset_sec", 0.0);
    use_stamped_msg_ = this->declare_parameter<bool>("use_stamped_msg", false);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (use_stamped_msg_) {
      yaw_stamped_sub_ = this->create_subscription<sp_msgs::msg::Float32Stamped>(
        input_topic_, rclcpp::QoS(10),
        std::bind(&AutoAimYawTfBroadcaster::onYawStamped, this, std::placeholders::_1));
    } else {
      yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        input_topic_, rclcpp::QoS(10),
        std::bind(&AutoAimYawTfBroadcaster::onYaw, this, std::placeholders::_1));
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
        std::bind(&AutoAimYawTfBroadcaster::publishLast, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Broadcasting TF '%s' -> '%s' from '%s' (std_msgs/Float32)",
      parent_frame_.c_str(), child_frame_.c_str(), input_topic_.c_str());
  }

private:
  void onYaw(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_yaw_rad_ = convertToRad(static_cast<double>(msg->data));
    have_last_ = true;
    last_stamp_ = this->get_clock()->now();
    publish(last_yaw_rad_, last_stamp_);
  }

  void onYawStamped(const sp_msgs::msg::Float32Stamped::SharedPtr msg)
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
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = (stamp + rclcpp::Duration::from_seconds(stamp_offset_sec_));
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;

    t.transform.translation.x = translation_x_;
    t.transform.translation.y = translation_y_;
    t.transform.translation.z = translation_z_;

    const double half = yaw_rad * 0.5;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(half);
    t.transform.rotation.w = std::cos(half);

    tf_broadcaster_->sendTransform(t);
  }

  std::string input_topic_;
  std::string parent_frame_;
  std::string child_frame_;

  double translation_x_{0.0};
  double translation_y_{0.0};
  double translation_z_{0.0};

  bool input_is_degrees_{false};
  bool invert_sign_{false};
  double scale_{1.0};
  double offset_{0.0};
  double publish_rate_hz_{0.0};
  double default_yaw_rad_{0.0};
  bool publish_on_startup_{true};
  double stamp_offset_sec_{0.0};
  bool use_stamped_msg_{false};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<sp_msgs::msg::Float32Stamped>::SharedPtr yaw_stamped_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_last_{false};
  double last_yaw_rad_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace gimbal_yaw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_yaw_bridge::AutoAimYawTfBroadcaster>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
