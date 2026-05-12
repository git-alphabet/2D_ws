#include "rm_behavior_tree/plugins/rmul_2026/action/move_around.hpp"

#include <random>
#include <rclcpp/clock.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

namespace rm_behavior_tree
{

MoveAroundAction::MoveAroundAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), Node("move_around_node")
{
  publisher_goal_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
}

BT::NodeStatus MoveAroundAction::onStart()
{
  current_pose_x = 0.0;
  current_pose_y = 0.0;
  current_orientation.x = 0.0;
  current_orientation.y = 0.0;
  current_orientation.z = 0.0;
  current_orientation.w = 1.0;
  expected_dis = 0.0;
  expected_nearby_goal_count = 0;
  goal_count = 0;

  // Prefer scalar pose inputs. Keep the legacy TransformStamped port for existing RMUL XML.
  const bool has_pose_xy = getInput("pose_x", current_pose_x) && getInput("pose_y", current_pose_y);
  if (!has_pose_xy) {
    geometry_msgs::msg::TransformStamped current_location;
    if (!getInput("message", current_location)) {
       std::cout << "missing required input [pose_x/pose_y] or [message]" << '\n';
      return BT::NodeStatus::FAILURE;
    }
    current_pose_x = current_location.transform.translation.x;
    current_pose_y = current_location.transform.translation.y;
    current_orientation = current_location.transform.rotation;
  }

  // 获取参数：期望的距离
  if (!getInput("expected_dis", expected_dis)) {
     std::cout << "missing required input [expected_dis]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  // 获取参数：期望的点位数量
  if (!getInput("expected_nearby_goal_count", expected_nearby_goal_count)) {
     std::cout << "missing required input [expected_nearby_goal_count]" << '\n';
    return BT::NodeStatus::FAILURE;
  }

  if (expected_nearby_goal_count <= 0) {
    // No need to go into the RUNNING state
    return BT::NodeStatus::SUCCESS;
  } else {
    // once the expected_nearby_goal_count is reached, we will return SUCCESS.
    last_goal_time_ = std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(1000);  // allow immediate first goal
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus MoveAroundAction::onRunning()
{
   std::cout << "expected_nearby_goal_count: " << expected_nearby_goal_count << '\n';
   std::cout << "goal_count: " << goal_count << '\n';
   std::cout << "expected_dis: " << expected_dis << '\n';

  if (expected_nearby_goal_count == goal_count) {
    return BT::NodeStatus::SUCCESS;
  } else {
    // Non-blocking: check if enough time has passed since last goal
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<milliseconds>(now - last_goal_time_).count();
    if (elapsed < 1000) {
      return BT::NodeStatus::RUNNING;  // Wait without blocking the BT thread
    }
    goal_count++;
    generatePoints(current_pose_x, current_pose_y, expected_dis, nearby_random_point);
    sendGoalPose(nearby_random_point);
    last_goal_time_ = now;
    return BT::NodeStatus::RUNNING;
  }
}

void MoveAroundAction::onHalted()
{
   //nothing to do here...
   std::cout << "MoveAroundAction interrupted" << '\n';
}

void MoveAroundAction::generatePoints(
  double pose_x, double pose_y, double distance, geometry_msgs::msg::PoseStamped & nearby_random_point)
{
  // 创建随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 2 * M_PI);

  // 生成随机角度
  double angle = dis(gen);

  nearby_random_point.header.stamp = rclcpp::Clock().now();
  nearby_random_point.header.frame_id = "map";
  nearby_random_point.pose.position.x = pose_x + distance * sin(angle);
  nearby_random_point.pose.position.y = pose_y + distance * cos(angle);
  nearby_random_point.pose.position.z = 0.0;
  nearby_random_point.pose.orientation = current_orientation;
}

void MoveAroundAction::sendGoalPose(geometry_msgs::msg::PoseStamped & msg)
{
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = nearby_random_point.header.frame_id;
  msg.pose.position.x = nearby_random_point.pose.position.x;
  msg.pose.position.y = nearby_random_point.pose.position.y;
  msg.pose.position.z = nearby_random_point.pose.position.z;
  msg.pose.orientation.x = nearby_random_point.pose.orientation.x;
  msg.pose.orientation.y = nearby_random_point.pose.orientation.y;
  msg.pose.orientation.z = nearby_random_point.pose.orientation.z;
  msg.pose.orientation.w = nearby_random_point.pose.orientation.w;

  // DEBUG
   std::cout << "pose.position.x: " << msg.pose.position.x << '\n';
   std::cout << "pose.position.y: " << msg.pose.position.y << '\n';
   std::cout << "pose.position.z: " << msg.pose.position.z << '\n';
   std::cout << "pose.orientation.x: " << msg.pose.orientation.x << '\n';
   std::cout << "pose.orientation.y: " << msg.pose.orientation.y << '\n';
   std::cout << "pose.orientation.z: " << msg.pose.orientation.z << '\n';
   std::cout << "pose.orientation.w: " << msg.pose.orientation.w << '\n';
   std::cout << "----------------" << '\n';

  publisher_goal_pose->publish(msg);
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::MoveAroundAction>("MoveAround");
}
