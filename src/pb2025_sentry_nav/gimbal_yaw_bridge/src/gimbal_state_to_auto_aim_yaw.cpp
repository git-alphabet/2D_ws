#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gimbal_yaw_interfaces/msg/float32_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace gimbal_yaw_bridge
{

class GimbalStateToAutoAimYaw final : public rclcpp::Node
{
public:
  explicit GimbalStateToAutoAimYaw(const rclcpp::NodeOptions & options)
  : Node("gimbal_state_to_auto_aim_yaw", options)
  {
    use_tf_source_ = this->declare_parameter<bool>("use_tf_source", true);
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "chassis");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "gimbal_yaw");

    input_topic_ = this->declare_parameter<std::string>("input_topic", "robot_base/gimbal_state");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "auto_aim_yaw");
    use_stamped_msg_ = this->declare_parameter<bool>("use_stamped_msg", false);

    invert_sign_ = this->declare_parameter<bool>("invert_sign", false);
    scale_ = this->declare_parameter<double>("scale", 1.0);
    offset_ = this->declare_parameter<double>("offset", 0.0);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    publish_on_startup_ = this->declare_parameter<bool>("publish_on_startup", false);
    default_yaw_rad_ = this->declare_parameter<double>("default_yaw_rad", 0.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");

    // 同名 topic 不能同时以不同 message type 创建 publisher。
    // 这里只按 use_stamped_msg_ 选择其一。
    if (use_stamped_msg_) {
      yaw_stamped_pub_ = this->create_publisher<gimbal_yaw_interfaces::msg::Float32Stamped>(
        output_topic_, rclcpp::QoS(10));
    } else {
      yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_topic_, rclcpp::QoS(10));
    }

    if (use_tf_source_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    } else {
      sub_ = this->create_subscription<rmoss_interfaces::msg::Gimbal>(
        input_topic_, rclcpp::QoS(10),
        std::bind(&GimbalStateToAutoAimYaw::onGimbalState, this, std::placeholders::_1));
    }

    if (publish_on_startup_) {
      last_yaw_rad_ = default_yaw_rad_;
      have_last_ = true;
      publish(last_yaw_rad_, this->get_clock()->now());
    }

    if (publish_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        use_tf_source_ ? std::bind(&GimbalStateToAutoAimYaw::tickTf, this)
                       : std::bind(&GimbalStateToAutoAimYaw::publishLast, this));
    }

    if (use_tf_source_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing '%s' from TF yaw(%s->%s) (%s), rate=%.1f Hz",
        output_topic_.c_str(),
        parent_frame_.c_str(),
        child_frame_.c_str(),
        use_stamped_msg_ ? "Float32Stamped" : "std_msgs/Float32",
        publish_rate_hz_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing '%s' from '%s' (rmoss_interfaces/Gimbal) (%s)",
        output_topic_.c_str(),
        input_topic_.c_str(),
        use_stamped_msg_ ? "Float32Stamped" : "std_msgs/Float32");
    }
  }

private:
  void onGimbalState(const rmoss_interfaces::msg::Gimbal::SharedPtr msg)
  {
    double yaw = static_cast<double>(msg->yaw);
    yaw = normalizeRad(yaw * scale_ + offset_);
    if (invert_sign_) {
      yaw = normalizeRad(-yaw);
    }

    last_yaw_rad_ = yaw;
    have_last_ = true;
    publish(last_yaw_rad_, this->get_clock()->now());
  }

  void tickTf()
  {
    if (!tf_buffer_) {
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (%s->%s): %s",
        parent_frame_.c_str(), child_frame_.c_str(), ex.what());
      return;
    }

    const auto & q = t.transform.rotation;
    // yaw (Z) from quaternion (x,y,z,w)
    const double siny_cosp = 2.0 * (static_cast<double>(q.w) * static_cast<double>(q.z) +
                    static_cast<double>(q.x) * static_cast<double>(q.y));
    const double cosy_cosp = 1.0 - 2.0 * (static_cast<double>(q.y) * static_cast<double>(q.y) +
                       static_cast<double>(q.z) * static_cast<double>(q.z));
    const double yaw_raw = std::atan2(siny_cosp, cosy_cosp);
    double yaw = normalizeRad(yaw_raw * scale_ + offset_);
    if (invert_sign_) {
      yaw = normalizeRad(-yaw);
    }

    last_yaw_rad_ = yaw;
    have_last_ = true;

    // 用 TF 自带 stamp（仿真时钟一致），避免 now 带来的跳变
    const rclcpp::Time stamp(t.header.stamp, this->get_clock()->get_clock_type());
    publish(last_yaw_rad_, stamp);
  }

  void publishLast()
  {
    if (!have_last_) {
      return;
    }
    publish(last_yaw_rad_, this->get_clock()->now());
  }

  static double normalizeRad(double angle)
  {
    // wrap to [-pi, pi]
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void publish(double yaw_rad, const rclcpp::Time & stamp)
  {
    if (use_stamped_msg_) {
      if (!yaw_stamped_pub_) {
        return;
      }
      gimbal_yaw_interfaces::msg::Float32Stamped out;
      out.header.stamp = stamp;
      out.header.frame_id = frame_id_;
      out.data = static_cast<float>(yaw_rad);
      yaw_stamped_pub_->publish(out);
      return;
    }

    if (!yaw_pub_) {
      return;
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(yaw_rad);
    yaw_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  bool use_tf_source_{true};
  std::string parent_frame_;
  std::string child_frame_;

  bool use_stamped_msg_{false};
  bool invert_sign_{false};
  double scale_{1.0};
  double offset_{0.0};

  double publish_rate_hz_{0.0};
  bool publish_on_startup_{false};
  double default_yaw_rad_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<gimbal_yaw_interfaces::msg::Float32Stamped>::SharedPtr yaw_stamped_pub_;
  rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool have_last_{false};
  double last_yaw_rad_{0.0};
};

}  // namespace gimbal_yaw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_yaw_bridge::GimbalStateToAutoAimYaw>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
