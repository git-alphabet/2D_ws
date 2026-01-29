#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace gimbal_yaw_bridge
{

class JointStateToFloat32Bridge final : public rclcpp::Node
{
public:
  explicit JointStateToFloat32Bridge(const rclcpp::NodeOptions & options)
  : Node("joint_state_to_float32_bridge", options)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "joint_states");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "auto_aim_yaw");
    joint_name_ = this->declare_parameter<std::string>("joint_name", "gimbal_yaw_joint");

    pub_ = this->create_publisher<std_msgs::msg::Float32>(output_topic_, rclcpp::QoS(10));
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&JointStateToFloat32Bridge::onJointState, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Bridging '%s' (sensor_msgs/JointState) -> '%s' (std_msgs/Float32), joint_name='%s'",
      input_topic_.c_str(), output_topic_.c_str(), joint_name_.c_str());
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() != msg->position.size()) {
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == joint_name_) {
        std_msgs::msg::Float32 out;
        out.data = static_cast<float>(msg->position[i]);
        pub_->publish(out);
        return;
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string joint_name_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace gimbal_yaw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_yaw_bridge::JointStateToFloat32Bridge>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
