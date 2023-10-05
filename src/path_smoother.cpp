/*
 * MIT License
 *
 * Copyright (c) 2023 CihatAltiparmak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <hybrid_astar_planner/path_smoother.hpp>

namespace planning
{

PathSmoother::PathSmoother(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("weight_obstacle", 0.5);
  node->get_parameter("weight_obstacle", Wobst_);

  node->declare_parameter("weight_smoothness", 0.2);
  node->get_parameter("weight_smoothness", Wsmoothness_);

  node->declare_parameter("iteration_number", 5000);
  node->get_parameter("iteration_number", iterationNumber_);

  node->declare_parameter("alpha", 0.0001);
  node->get_parameter("alpha", alpha_);

  node->declare_parameter("dist_max", 3.0);
  node->get_parameter("dist_max", distMax_);
}

void PathSmoother::doSettingsWithMap(const grid_map::GridMap & map)
{
  std::vector<KDPoint> obstacles;

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    if (map.at("obstacle", *it) > 0.0) {
      grid_map::Position obstacle_pos;
      map.getPosition(*it, obstacle_pos);
      obstacles.push_back({obstacle_pos.x(), obstacle_pos.y()});
    }
  }
  kdTree_ = KDTree(obstacles, 2);
}

planner_msgs::msg::Path PathSmoother::smoothPath(
  const std::vector<Vector2d> & path)
{

  // if the path is empty, no need to smooth path
  if (path.empty()) {
    auto msg = planner_msgs::msg::Path();
    msg.header.frame_id = "map";
    return msg;
  }

  std::vector<Vector2d> new_path = path;

  for (int i = 0; i < iterationNumber_; i++) {
    Vector2d correction(0.0, 0.0);

    for (int i = 2; i < static_cast<int>(new_path.size()) - 2; i++) {
      correction =
        Wsmoothness_ * smoothingTerm(
        new_path[i - 2], new_path[i - 1],
        new_path[i], new_path[i + 1],
        new_path[i + 2]);
      new_path[i] -= alpha_ * correction;
    }

    for (int i = 1; i < static_cast<int>(new_path.size()) - 1; i++) {
      correction = Wobst_ * obstacleTerm(new_path[i]);
      new_path[i] -= alpha_ * correction;
    }
  }

  planner_msgs::msg::Path smoothed_path_msg;
  smoothed_path_msg.header.frame_id = "map";
  for (int i = 0; i < static_cast<int>(new_path.size()) - 1; i++) {
    planner_msgs::msg::Point point;
    point.x = new_path[i][0];
    point.y = new_path[i][1];
    point.yaw = std::atan2(
      (new_path[i + 1][1] - new_path[i][1]),
      (new_path[i + 1][0] - new_path[i][0]));

    smoothed_path_msg.points.push_back(point);
  }

  planner_msgs::msg::Point point;
  point.x = new_path.back()[0];
  point.y = new_path.back()[1];
  point.yaw = 0.0;

  smoothed_path_msg.points.push_back(point);
  return smoothed_path_msg;
}

Vector2d PathSmoother::obstacleTerm(const Vector2d & pointVect)
{
  Vector2d nearestObstVect =
    Vector2d(kdTree_.getNearestPoint({pointVect[0], pointVect[1]}));
  Vector2d obstVector = pointVect - nearestObstVect;
  double dist = obstVector.norm2();
  double distMax_ = 3;

  if (dist < distMax_) {
    return (2 * (dist - distMax_) * obstVector) / dist;
  }

  return Vector2d(0.0, 0.0);
}

Vector2d PathSmoother::smoothingTerm(
  const Vector2d & xim2, const Vector2d & xim1,
  const Vector2d & xi, const Vector2d & xip1,
  const Vector2d & xip2)
{
  return 2 * (xip2 - 4 * xip1 + 6 * xi - 4 * xim1 + xim2);
}

}  // end of namespace planning
