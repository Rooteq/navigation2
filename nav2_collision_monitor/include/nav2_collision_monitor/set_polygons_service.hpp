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

#ifndef NAV2_COLLISION_MONITOR__SET_POLYGONS_SERVICE_HPP_
#define NAV2_COLLISION_MONITOR__SET_POLYGONS_SERVICE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_collision_monitor_msgs/srv/set_polygons.hpp"

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Converts polygon vertices into the "<polygon>.points" parameter format:
 * "[[x1, y1], [x2, y2], ...]"
 * @param polygon Input polygon
 * @return Vertices string
 */
std::string polygonToPointsString(const geometry_msgs::msg::Polygon & polygon);

/**
 * @brief Handles a SetPolygons request.
 * Writes the "<polygon>.points" and "<polygon>.enabled" parameters of the node in one
 * atomic call, so the Polygon parameter callbacks check and apply the new shapes,
 * and the parameters always show the shapes in use.
 * @param node Node owning the polygons
 * @param polygons Polygons of the node
 * @param request Service request
 * @param response Service response
 */
void handleSetPolygons(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::vector<std::shared_ptr<Polygon>> & polygons,
  const nav2_collision_monitor_msgs::srv::SetPolygons::Request & request,
  nav2_collision_monitor_msgs::srv::SetPolygons::Response & response);

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SET_POLYGONS_SERVICE_HPP_
