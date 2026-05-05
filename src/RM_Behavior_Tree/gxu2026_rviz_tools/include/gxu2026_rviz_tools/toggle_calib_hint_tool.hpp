#ifndef GXU2026_RVIZ_TOOLS__TOGGLE_CALIB_HINT_TOOL_HPP_
#define GXU2026_RVIZ_TOOLS__TOGGLE_CALIB_HINT_TOOL_HPP_

#include <memory>
#include <string>

#include <QObject>

#include "rclcpp/node.hpp"
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
}
}

namespace gxu2026_rviz_tools
{

class ToggleCalibHintTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  ToggleCalibHintTool();
  ~ToggleCalibHintTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

private Q_SLOTS:
  void updateTopics();

private:
  void publishDesiredState(bool visible);
  void updateAppearance();
  void scheduleReturnToDefaultTool();
  std::string iconPath(const char * file_name) const;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;

  rviz_common::properties::StringProperty * command_topic_property_;
  rviz_common::properties::StringProperty * state_topic_property_;

  bool hint_visible_{true};
  bool have_state_{false};
};

}  // namespace gxu2026_rviz_tools

#endif  // GXU2026_RVIZ_TOOLS__TOGGLE_CALIB_HINT_TOOL_HPP_
