// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_
#define NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "nav2_neupan_controller/neupan_types.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"

namespace Json
{
class Value;
}

namespace nav2_neupan_controller
{

// 与独立 neupan-runtime 容器通信的桥接层。
class PythonBridge
{
public:
  explicit PythonBridge(const rclcpp::Logger & logger);
  ~PythonBridge() = default;

  bool initialize(const std::string & config_path, const std::string & dune_model_path);
  void cleanup();

  bool isInitialized() const { return initialized_; }
  const RobotInfo & robotInfo() const { return robot_info_; }

  bool callForward(
    const std::array<double, 3> & robot_state,
    const std::vector<std::pair<double, double>> & obstacles, const std::string & robot_type,
    PlannerOutput & output);

  bool callReset();
  bool setInitialPath(const std::vector<NeuPANWaypoint> & path);
  bool updatePathFromGoal(const std::array<double, 3> & start, const std::array<double, 3> & goal);

private:
  bool postJson(const std::string & endpoint, const std::string & body, std::string & response);
  bool parseOkResponse(const std::string & response, Json::Value & root, std::string & error_msg) const;

  void decodeAction(
    const Json::Value & action_json, const std::string & robot_type,
    geometry_msgs::msg::Twist & cmd_vel);

  static std::vector<std::array<double, 3>> parseStates(const Json::Value & states_json);
  static std::vector<std::pair<double, double>> parsePoints(const Json::Value & points_json);
  static std::vector<double> extractNumbers(const Json::Value & value);
  static std::string trim(const std::string & value);

  static size_t writeCallback(void * contents, size_t size, size_t nmemb, void * userp);

  rclcpp::Logger logger_;
  bool initialized_{false};
  RobotInfo robot_info_;
  std::string service_url_;
  long timeout_ms_{500};
};

}  // namespace nav2_neupan_controller

#endif  // NAV2_NEUPAN_CONTROLLER__PYTHON_BRIDGE_HPP_
