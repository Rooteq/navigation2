// Copyright (c) 2026 Rooteq
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

#include "nav2_collision_monitor/set_polygons_service.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <exception>
#include <unordered_set>

namespace nav2_collision_monitor
{

// Shortest text that reads back as the same float: 0.4f -> "0.4", not "0.400000006"
static std::string floatToString(const float value)
{
  std::array<char, 32> buf;
  const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  return std::string(buf.data(), res.ptr);
}

std::string polygonToPointsString(const geometry_msgs::msg::Polygon & polygon)
{
  std::string points = "[";
  for (std::size_t i = 0; i < polygon.points.size(); i++) {
    if (i > 0) {
      points += ", ";
    }
    points += "[" + floatToString(polygon.points[i].x) + ", " +
      floatToString(polygon.points[i].y) + "]";
  }
  points += "]";
  return points;
}

void handleSetPolygons(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::vector<std::shared_ptr<Polygon>> & polygons,
  const nav2_collision_monitor_msgs::srv::SetPolygons::Request & request,
  nav2_collision_monitor_msgs::srv::SetPolygons::Response & response)
{
  response.success = false;

  if (request.polygons.empty()) {
    response.message = "Request contains no polygons";
    RCLCPP_WARN(node.get_logger(), "SetPolygons rejected: %s", response.message.c_str());
    return;
  }

  std::vector<rclcpp::Parameter> parameters;
  std::unordered_set<std::string> names;
  for (const auto & named_polygon : request.polygons) {
    const std::string & name = named_polygon.name;

    if (!names.insert(name).second) {
      response.message = "Polygon '" + name + "' is given more than once";
    } else if (
      std::none_of(
        polygons.begin(), polygons.end(),
        [&name](const std::shared_ptr<Polygon> & polygon) {return polygon->getName() == name;}))
    {
      response.message = "Unknown polygon '" + name + "'";
    } else if (node.get_parameter(name + ".type").as_string() != "polygon") {
      response.message =
        "Polygon '" + name + "' is not of type \"polygon\", its shape cannot be set";
    } else if (
      std::any_of(
        named_polygon.polygon.points.begin(), named_polygon.polygon.points.end(),
        [](const geometry_msgs::msg::Point32 & p) {
          return !std::isfinite(p.x) || !std::isfinite(p.y);
        }))
    {
      response.message = "Polygon '" + name + "' has a non-finite vertex";
    }

    if (!response.message.empty()) {
      RCLCPP_WARN(node.get_logger(), "SetPolygons rejected: %s", response.message.c_str());
      return;
    }

    // A polygon switched off without vertices keeps the ones it has
    if (named_polygon.enabled || !named_polygon.polygon.points.empty()) {
      parameters.emplace_back(name + ".points", polygonToPointsString(named_polygon.polygon));
    }
    parameters.emplace_back(name + ".enabled", static_cast<bool>(named_polygon.enabled));
  }

  try {
    const rcl_interfaces::msg::SetParametersResult result =
      node.set_parameters_atomically(parameters);
    response.success = result.successful;
    response.message = result.successful ?
      "Updated " + std::to_string(request.polygons.size()) + " polygon(s)" : result.reason;
  } catch (const std::exception & ex) {
    response.message = ex.what();
  }

  if (!response.success) {
    RCLCPP_WARN(node.get_logger(), "SetPolygons rejected: %s", response.message.c_str());
  }
}

}  // namespace nav2_collision_monitor
