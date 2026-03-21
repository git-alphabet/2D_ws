#include "rm_behavior_tree/plugins/rmul_2026/action/sub_robot_position.hpp"

#include "behaviortree_ros2/plugins.hpp"  // CreateRosNodePlugin 宏
#include <array>

namespace rm_behavior_tree
{

SubRobotPositionAction::SubRobotPositionAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)
{
  if (!node_) {
    throw std::runtime_error("SubRobotPositionAction: ROS node is null");
  }

  // 初始化 TF 监听器，作为获取机器人真实位置的备用/首选途径
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string topic;
  // 使用模板形式以符合 BehaviorTree::TreeNode API
  if (!getInput<std::string>("topic_name", topic)) {
    // 没有输入时使用默认话题（PortsList 中也设置了默认），并记录警告
    RCLCPP_WARN(node_->get_logger(), "SubRobotPositionAction: no topic_name input provided, using default '%s'", topic.c_str());
  }

  // 创建订阅：QoS 缓冲 10，可靠传输（可按需调整）
  rclcpp::QoS qos(10);
  qos.reliable();

  sub_ = node_->create_subscription<rm_decision_interfaces::msg::RMUL>(
    topic,
    qos,
    [this](const rm_decision_interfaces::msg::RMUL::SharedPtr msg) {
      this->robot_position_callback(msg);
    });
}

void SubRobotPositionAction::robot_position_callback(
  const rm_decision_interfaces::msg::RMUL::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pose_x_ = msg->x;
  pose_y_ = msg->y;

  // 记录接收时间（如果消息带时间戳并且你希望使用它，可以改为使用 msg->header.stamp）
  last_stamp_ = node_->now();

  has_data_ = true;
}

BT::NodeStatus SubRobotPositionAction::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 从 TF 中获取位置（优先，更准更稳）
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    std::string ns = node_->get_namespace();
    if (!ns.empty() && ns != "/") {
      if (ns[0] == '/') {
        ns = ns.substr(1);
      }
      ns += "/";
    } else {
      ns.clear();
    }

    bool tf_ok = false;
    const std::array<std::pair<std::string, std::string>, 4> candidates = {{
      {"map", ns + "base_footprint"},
      {"map", "base_footprint"},
      {"map", ns + "base_link"},
      {"map", "base_link"}
    }};

    for (const auto & candidate : candidates) {
      try {
        transform_stamped = tf_buffer_->lookupTransform(candidate.first, candidate.second, tf2::TimePointZero);
        tf_ok = true;
        break;
      } catch (const tf2::TransformException &) {
      }
    }

    if (!tf_ok) {
      throw tf2::TransformException("No valid TF in map frame to base frames");
    }

    pose_x_ = transform_stamped.transform.translation.x;
    pose_y_ = transform_stamped.transform.translation.y;
    has_data_ = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "SubRobotPositionAction TF failed: %s, falling back to topic", ex.what());
  }

  if (!has_data_) {
    // 尚未收到裁判系统数据且 TF 获取失败，不阻塞行为树，返回 SUCCESS
    return BT::NodeStatus::SUCCESS;
  }

  // 将最新位置写入输出端口
  setOutput("pose_x", pose_x_);
  setOutput("pose_y", pose_y_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

// 将节点作为插件导出（放在全局作用域）
CreateRosNodePlugin(
  rm_behavior_tree::SubRobotPositionAction,
  "SubRobotPosition");
