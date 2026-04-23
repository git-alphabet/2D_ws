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

#include "nav2_neupan_controller/python_bridge.hpp"

#include <curl/curl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "jsoncpp/json/json.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace nav2_neupan_controller
{

namespace
{

std::string trimCopy(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool isMatrix3xN(const Json::Value & value)
{
  if (!value.isArray() || value.size() != 3U) {
    return false;
  }
  if (!value[0].isArray() || !value[1].isArray() || !value[2].isArray()) {
    return false;
  }
  const auto n0 = value[0].size();
  const auto n1 = value[1].size();
  const auto n2 = value[2].size();
  if (n0 <= 1U || n0 != n1 || n0 != n2) {
    return false;
  }
  for (Json::ArrayIndex i = 0; i < n0; ++i) {
    if (!value[0][i].isNumeric() || !value[1][i].isNumeric() || !value[2][i].isNumeric()) {
      return false;
    }
  }
  return true;
}

bool isMatrix2xN(const Json::Value & value)
{
  if (!value.isArray() || value.size() != 2U) {
    return false;
  }
  if (!value[0].isArray() || !value[1].isArray()) {
    return false;
  }
  const auto n0 = value[0].size();
  const auto n1 = value[1].size();
  if (n0 <= 1U || n0 != n1) {
    return false;
  }
  for (Json::ArrayIndex i = 0; i < n0; ++i) {
    if (!value[0][i].isNumeric() || !value[1][i].isNumeric()) {
      return false;
    }
  }
  return true;
}

bool extractSingleState(const Json::Value & value, std::array<double, 3> & state)
{
  if (!value.isArray() || value.size() < 3U) {
    return false;
  }

  if (value[0].isNumeric() && value[1].isNumeric() && value[2].isNumeric()) {
    state = {value[0].asDouble(), value[1].asDouble(), value[2].asDouble()};
    return true;
  }

  if (
    value[0].isArray() && value[1].isArray() && value[2].isArray() &&
    value[0].size() >= 1U && value[1].size() >= 1U && value[2].size() >= 1U &&
    value[0][0].isNumeric() && value[1][0].isNumeric() && value[2][0].isNumeric()) {
    state = {value[0][0].asDouble(), value[1][0].asDouble(), value[2][0].asDouble()};
    return true;
  }

  return false;
}

bool extractSinglePoint(const Json::Value & value, std::pair<double, double> & point)
{
  if (!value.isArray() || value.size() < 2U) {
    return false;
  }

  if (value[0].isNumeric() && value[1].isNumeric()) {
    point = {value[0].asDouble(), value[1].asDouble()};
    return true;
  }

  if (
    value[0].isArray() && value[1].isArray() &&
    value[0].size() >= 1U && value[1].size() >= 1U &&
    value[0][0].isNumeric() && value[1][0].isNumeric()) {
    point = {value[0][0].asDouble(), value[1][0].asDouble()};
    return true;
  }

  return false;
}

long parseTimeoutMs()
{
  constexpr long default_timeout_ms = 500L;
  const char * raw = std::getenv("NEUPAN_SERVICE_TIMEOUT_MS");
  if (!raw) {
    return default_timeout_ms;
  }

  try {
    const long value = std::stol(trimCopy(raw));
    return value > 0L ? value : default_timeout_ms;
  } catch (...) {
    return default_timeout_ms;
  }
}

std::string parseServiceUrl()
{
  const char * raw = std::getenv("NEUPAN_SERVICE_URL");
  std::string url = raw ? trimCopy(raw) : "http://127.0.0.1:18080";
  if (url.empty()) {
    url = "http://127.0.0.1:18080";
  }
  while (!url.empty() && url.back() == '/') {
    url.pop_back();
  }
  return url;
}

}  // namespace

PythonBridge::PythonBridge(const rclcpp::Logger & logger)
: logger_(logger), service_url_(parseServiceUrl()), timeout_ms_(parseTimeoutMs())
{
  (void)curl_global_init(CURL_GLOBAL_DEFAULT);
}

bool PythonBridge::initialize(const std::string & config_path, const std::string & dune_model_path)
{
  if (initialized_) {
    return true;
  }

  if (config_path.empty()) {
    RCLCPP_ERROR(logger_, "neupan_config_path is empty");
    return false;
  }

  Json::Value req(Json::objectValue);
  req["config_path"] = config_path;
  if (!dune_model_path.empty()) {
    req["dune_model_path"] = dune_model_path;
  }

  Json::StreamWriterBuilder writer;
  std::string body = Json::writeString(writer, req);
  std::string response;
  if (!postJson("/init", body, response)) {
    return false;
  }

  Json::Value root;
  std::string error_msg;
  if (!parseOkResponse(response, root, error_msg)) {
    RCLCPP_ERROR(logger_, "neupan /init failed: %s", error_msg.c_str());
    return false;
  }

  initialized_ = true;
  RCLCPP_INFO(logger_, "NeuPAN runtime service initialized via %s", service_url_.c_str());
  return true;
}

void PythonBridge::cleanup()
{
  initialized_ = false;
}

bool PythonBridge::callForward(
  const std::array<double, 3> & robot_state,
  const std::vector<std::pair<double, double>> & obstacles, const std::string & robot_type,
  PlannerOutput & output)
{
  if (!initialized_) {
    return false;
  }

  Json::Value req(Json::objectValue);
  Json::Value state(Json::arrayValue);
  state.append(robot_state[0]);
  state.append(robot_state[1]);
  state.append(robot_state[2]);
  req["robot_state"] = state;

  Json::Value obs(Json::arrayValue);
  for (const auto & p : obstacles) {
    Json::Value pair(Json::arrayValue);
    pair.append(p.first);
    pair.append(p.second);
    obs.append(pair);
  }
  req["obstacles"] = obs;

  Json::StreamWriterBuilder writer;
  const std::string body = Json::writeString(writer, req);

  std::string response;
  if (!postJson("/forward", body, response)) {
    return false;
  }

  Json::Value root;
  std::string error_msg;
  if (!parseOkResponse(response, root, error_msg)) {
    RCLCPP_ERROR(logger_, "neupan /forward failed: %s", error_msg.c_str());
    return false;
  }

  output.stop = root.get("stop", false).asBool();
  output.arrive = root.get("arrive", false).asBool();

  if (output.stop) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(), 500,
      "NeuPAN stopped - collision threshold reached");
  }
  if (output.arrive) {
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 100, "NeuPAN arrived at target");
  }

  if (!output.stop && !output.arrive) {
    decodeAction(root["action"], robot_type, output.cmd_vel);
  }

  output.opt_states = parseStates(root["opt_state_list"]);
  output.ref_states = parseStates(root["ref_state_list"]);
  output.initial_path = parseStates(root["initial_path"]);
  output.dune_pts = parsePoints(root["dune_points"]);
  output.nrmp_pts = parsePoints(root["nrmp_points"]);

  const Json::Value & robot = root["robot"];
  if (robot.isObject()) {
    robot_info_.shape = robot.get("shape", robot_info_.shape).asString();
    robot_info_.kinematics = robot.get("kinematics", robot_info_.kinematics).asString();
    robot_info_.length = robot.get("length", robot_info_.length).asDouble();
    robot_info_.width = robot.get("width", robot_info_.width).asDouble();
    robot_info_.wheelbase = robot.get("wheelbase", robot_info_.wheelbase).asDouble();
  }

  return true;
}

bool PythonBridge::callReset()
{
  if (!initialized_) {
    return false;
  }

  std::string response;
  if (!postJson("/reset", "{}", response)) {
    return false;
  }

  Json::Value root;
  std::string error_msg;
  if (!parseOkResponse(response, root, error_msg)) {
    RCLCPP_WARN(logger_, "neupan /reset failed: %s", error_msg.c_str());
    return false;
  }
  return true;
}

bool PythonBridge::setInitialPath(const std::vector<NeuPANWaypoint> & path)
{
  if (!initialized_) {
    return false;
  }

  Json::Value req(Json::objectValue);
  Json::Value points(Json::arrayValue);
  for (const auto & wp : path) {
    Json::Value one(Json::arrayValue);
    one.append(wp.x);
    one.append(wp.y);
    one.append(wp.theta);
    one.append(wp.gear);
    points.append(one);
  }
  req["path"] = points;

  Json::StreamWriterBuilder writer;
  const std::string body = Json::writeString(writer, req);

  std::string response;
  if (!postJson("/set_initial_path", body, response)) {
    return false;
  }

  Json::Value root;
  std::string error_msg;
  if (!parseOkResponse(response, root, error_msg)) {
    RCLCPP_ERROR(logger_, "neupan /set_initial_path failed: %s", error_msg.c_str());
    return false;
  }
  return true;
}

bool PythonBridge::updatePathFromGoal(
  const std::array<double, 3> & start, const std::array<double, 3> & goal)
{
  if (!initialized_) {
    return false;
  }

  Json::Value req(Json::objectValue);
  Json::Value start_json(Json::arrayValue);
  start_json.append(start[0]);
  start_json.append(start[1]);
  start_json.append(start[2]);
  req["start"] = start_json;

  Json::Value goal_json(Json::arrayValue);
  goal_json.append(goal[0]);
  goal_json.append(goal[1]);
  goal_json.append(goal[2]);
  req["goal"] = goal_json;

  Json::StreamWriterBuilder writer;
  const std::string body = Json::writeString(writer, req);

  std::string response;
  if (!postJson("/update_initial_path_from_goal", body, response)) {
    return false;
  }

  Json::Value root;
  std::string error_msg;
  if (!parseOkResponse(response, root, error_msg)) {
    RCLCPP_WARN(logger_, "neupan /update_initial_path_from_goal failed: %s", error_msg.c_str());
    return false;
  }
  return true;
}

bool PythonBridge::postJson(const std::string & endpoint, const std::string & body, std::string & response)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    RCLCPP_ERROR(logger_, "curl_easy_init failed");
    return false;
  }

  const std::string url = service_url_ + endpoint;
  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  response.clear();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, timeout_ms_);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &PythonBridge::writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  const CURLcode rc = curl_easy_perform(curl);
  if (rc != CURLE_OK) {
    RCLCPP_ERROR(logger_, "HTTP request failed: %s", curl_easy_strerror(rc));
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return false;
  }

  long code = 0L;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (code != 200L) {
    RCLCPP_ERROR(logger_, "HTTP status %ld from %s, body=%s", code, url.c_str(), response.c_str());
    return false;
  }
  return true;
}

bool PythonBridge::parseOkResponse(
  const std::string & response, Json::Value & root, std::string & error_msg) const
{
  Json::CharReaderBuilder builder;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  std::string errs;
  const bool ok = reader->parse(
    response.data(), response.data() + response.size(), &root, &errs);
  if (!ok) {
    error_msg = "json parse failed: " + errs;
    return false;
  }

  if (!root.isObject()) {
    error_msg = "response is not a JSON object";
    return false;
  }

  if (!root.get("ok", false).asBool()) {
    error_msg = root.get("error", "unknown error").asString();
    return false;
  }

  return true;
}

void PythonBridge::decodeAction(
  const Json::Value & action_json, const std::string & robot_type,
  geometry_msgs::msg::Twist & cmd_vel)
{
  const auto numbers = extractNumbers(action_json);
  if (numbers.size() < 2U) {
    RCLCPP_WARN(logger_, "Invalid action payload from runtime");
    return;
  }

  if (robot_type == "omni") {
    cmd_vel.linear.x = numbers[0];
    cmd_vel.linear.y = numbers[1];
    cmd_vel.angular.z = 0.0;
  } else if (robot_type == "acker") {
    const double v = numbers[0];
    const double steering = numbers[1];
    const double wb = robot_info_.wheelbase;
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = (std::abs(v) > 1e-4) ? (v / wb) * std::tan(steering) : 0.0;
  } else {
    cmd_vel.linear.x = numbers[0];
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = numbers[1];
  }
}

std::vector<std::array<double, 3>> PythonBridge::parseStates(const Json::Value & states_json)
{
  std::vector<std::array<double, 3>> result;
  if (states_json.isNull() || !states_json.isArray()) {
    return result;
  }

  if (isMatrix3xN(states_json)) {
    const auto cols = states_json[0].size();
    result.reserve(cols);
    for (Json::ArrayIndex i = 0; i < cols; ++i) {
      result.push_back(
        {states_json[0][i].asDouble(), states_json[1][i].asDouble(), states_json[2][i].asDouble()});
    }
    return result;
  }

  result.reserve(states_json.size());
  for (Json::ArrayIndex i = 0; i < states_json.size(); ++i) {
    std::array<double, 3> state{};
    if (extractSingleState(states_json[i], state)) {
      result.push_back(state);
      continue;
    }

    if (isMatrix3xN(states_json[i])) {
      const auto cols = states_json[i][0].size();
      for (Json::ArrayIndex c = 0; c < cols; ++c) {
        result.push_back(
          {states_json[i][0][c].asDouble(), states_json[i][1][c].asDouble(),
            states_json[i][2][c].asDouble()});
      }
    }
  }
  return result;
}

std::vector<std::pair<double, double>> PythonBridge::parsePoints(const Json::Value & points_json)
{
  std::vector<std::pair<double, double>> result;
  if (points_json.isNull() || !points_json.isArray()) {
    return result;
  }

  if (isMatrix2xN(points_json)) {
    const auto cols = points_json[0].size();
    result.reserve(cols);
    for (Json::ArrayIndex i = 0; i < cols; ++i) {
      result.emplace_back(points_json[0][i].asDouble(), points_json[1][i].asDouble());
    }
    return result;
  }

  result.reserve(points_json.size());
  for (Json::ArrayIndex i = 0; i < points_json.size(); ++i) {
    std::pair<double, double> point{};
    if (extractSinglePoint(points_json[i], point)) {
      result.push_back(point);
      continue;
    }

    if (isMatrix2xN(points_json[i])) {
      const auto cols = points_json[i][0].size();
      for (Json::ArrayIndex c = 0; c < cols; ++c) {
        result.emplace_back(points_json[i][0][c].asDouble(), points_json[i][1][c].asDouble());
      }
    }
  }
  return result;
}

std::vector<double> PythonBridge::extractNumbers(const Json::Value & value)
{
  std::vector<double> out;
  if (value.isNumeric()) {
    out.push_back(value.asDouble());
    return out;
  }
  if (!value.isArray()) {
    return out;
  }

  for (Json::ArrayIndex i = 0; i < value.size(); ++i) {
    const auto nested = extractNumbers(value[i]);
    out.insert(out.end(), nested.begin(), nested.end());
  }
  return out;
}

std::string PythonBridge::trim(const std::string & value)
{
  return trimCopy(value);
}

size_t PythonBridge::writeCallback(void * contents, size_t size, size_t nmemb, void * userp)
{
  const size_t total = size * nmemb;
  auto * out = static_cast<std::string *>(userp);
  out->append(static_cast<const char *>(contents), total);
  return total;
}

}  // namespace nav2_neupan_controller
