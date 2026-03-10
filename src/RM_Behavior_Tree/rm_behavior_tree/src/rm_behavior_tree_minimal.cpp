#include "rm_behavior_tree/rm_behavior_tree.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree_minimal");
  node->declare_parameter<std::string>("style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>("style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/minimal_detect_respawn.xml");

  RCLCPP_INFO(node->get_logger(), "[minimal] Load bt_xml: %s", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg_minimal");
  params_update_msg.default_port_value = std::string("/robot_status");

  // Only register the minimal message-update plugins required
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_robot_status",
    "detect_respawn_and_set_recovery",
  };

  const std::vector<std::string> bt_plugin_libs = {
    // empty
  };

  for (const auto & p : msg_update_plugin_libs) {
    try {
      RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load msg-update plugin '%s': %s", p.c_str(), e.what());
    }
  }

  for (const auto & p : bt_plugin_libs) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load BT plugin '%s': %s", p.c_str(), e.what());
    }
  }

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create behavior tree from '%s': %s", bt_xml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Groot2Publisher disabled for minimal test to avoid ZMQ port conflicts
  // const unsigned port = 1668; // different port to avoid clash
  // BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
