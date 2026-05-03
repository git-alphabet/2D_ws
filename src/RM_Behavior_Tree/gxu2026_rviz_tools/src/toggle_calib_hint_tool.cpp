#include "gxu2026_rviz_tools/toggle_calib_hint_tool.hpp"

#include <memory>
#include <string>

#include <QIcon>
#include <QTimer>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/tool_manager.hpp"

namespace gxu2026_rviz_tools
{

ToggleCalibHintTool::ToggleCalibHintTool()
{
  shortcut_key_ = 'h';
  command_topic_property_ = new rviz_common::properties::StringProperty(
    "Command Topic", "/calib/show_hint/set",
    "Topic used to set the calibration hint visible state.",
    getPropertyContainer(), SLOT(updateTopics()), this);
  state_topic_property_ = new rviz_common::properties::StringProperty(
    "State Topic", "/calib/show_hint/state",
    "Latched topic reporting the current calibration hint visible state.",
    getPropertyContainer(), SLOT(updateTopics()), this);
}

void ToggleCalibHintTool::onInitialize()
{
  setName("Toggle Calib Hint");
  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setDescription("RViz ROS node unavailable; calib hint toggle disabled.");
    return;
  }

  raw_node_ = node_abstraction->get_raw_node();
  updateTopics();
  updateAppearance();
}

void ToggleCalibHintTool::activate()
{
  if (!command_pub_) {
    setStatus("Calib hint toggle unavailable: command publisher not ready.");
    scheduleReturnToDefaultTool();
    return;
  }

  const bool next_visible = have_state_ ? !hint_visible_ : false;
  publishDesiredState(next_visible);
  setStatus(next_visible ? "Calibration hint enabled" : "Calibration hint hidden");
  scheduleReturnToDefaultTool();
}

void ToggleCalibHintTool::deactivate()
{
}

void ToggleCalibHintTool::updateTopics()
{
  if (!raw_node_) {
    return;
  }

  command_pub_.reset();
  state_sub_.reset();

  const auto command_topic = command_topic_property_->getStdString();
  const auto state_topic = state_topic_property_->getStdString();

  command_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(
    command_topic, rclcpp::QoS(1).reliable());

  state_sub_ = raw_node_->create_subscription<std_msgs::msg::Bool>(
    state_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      have_state_ = true;
      hint_visible_ = msg->data;
      updateAppearance();
      if (context_ && context_->getToolManager()) {
        context_->getToolManager()->refreshTool(this);
      }
    });
}

void ToggleCalibHintTool::publishDesiredState(bool visible)
{
  std_msgs::msg::Bool msg;
  msg.data = visible;
  command_pub_->publish(msg);
  hint_visible_ = visible;
  have_state_ = true;
  updateAppearance();
  if (context_ && context_->getToolManager()) {
    context_->getToolManager()->refreshTool(this);
  }
}

void ToggleCalibHintTool::updateAppearance()
{
  const std::string icon_file = hint_visible_ ? "calib_hint_on.svg" : "calib_hint_off.svg";
  setIcon(QIcon(QString::fromStdString(iconPath(icon_file.c_str()))));
  setDescription(
    hint_visible_
      ? "Hide the center-screen calibration hint text."
      : "Show the center-screen calibration hint text.");
}

void ToggleCalibHintTool::scheduleReturnToDefaultTool()
{
  QTimer::singleShot(0, [this]() {
    if (!context_ || !context_->getToolManager()) {
      return;
    }
    auto * tool_manager = context_->getToolManager();
    auto * default_tool = tool_manager->getDefaultTool();
    if (default_tool && default_tool != this) {
      tool_manager->setCurrentTool(default_tool);
    }
  });
}

std::string ToggleCalibHintTool::iconPath(const char * file_name) const
{
  const auto share_dir = ament_index_cpp::get_package_share_directory("gxu2026_rviz_tools");
  return share_dir + "/icons/" + file_name;
}

}  // namespace gxu2026_rviz_tools

PLUGINLIB_EXPORT_CLASS(gxu2026_rviz_tools::ToggleCalibHintTool, rviz_common::Tool)
