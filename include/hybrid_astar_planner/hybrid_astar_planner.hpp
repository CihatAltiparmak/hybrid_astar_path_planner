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

#ifndef __HYBRID_ASTAR_PLANNER_HPP__
#define __HYBRID_ASTAR_PLANNER_HPP__

#include <chrono>
#include <cmath>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <hybrid_astar_planner/path_smoother.hpp>
#include <hybrid_astar_planner/utils/node.hpp>
#include <hybrid_astar_planner/utils/vector.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <planner_msgs/msg/path.hpp>
#include <planner_msgs/msg/point.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

namespace planning
{

// https://stackoverflow.com/a/14369745

class HybridAstarPlanner
{
public:
  HybridAstarPlanner() = default;
  HybridAstarPlanner(rclcpp::Node::SharedPtr);
  planner_msgs::msg::Path plan(
    std::shared_ptr<Node>,
    std::shared_ptr<Node>);
  void updateNeigbour(
    std::shared_ptr<Node>, std::shared_ptr<Node>,
    std::vector<std::shared_ptr<Node>> &,
    std::priority_queue<std::shared_ptr<Node>> &,
    std::vector<std::shared_ptr<Node>> &);
  void addToClosedList(const std::shared_ptr<Node> &, std::vector<std::shared_ptr<Node>> &);
  double heruisticCost(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &);
  void updateExistingNodeInCell(std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>> &);
  bool isPathValid(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &);
  bool isInsideOfMap(const std::shared_ptr<Node> &);
  bool isClosed(const std::shared_ptr<Node> &, std::vector<std::shared_ptr<Node>> &);
  bool isGoalReached(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &);
  grid_map::Index getIndexOfNode(const std::shared_ptr<Node> &);
  grid_map::Position getPositionOfNode(const std::shared_ptr<Node> &);

  grid_map::GridMap getMap();
  void setMap(grid_map::GridMap);

private:
  std::vector<double> steeringInputs_;
  std::vector<double> directionInputs_;
  double wheelbase_;
  int timeLimit_;
  int maxIterationNumber_;
  double goalTolerance_;
  double turningRadius_;

  double steeringCost_;
  double headingChangeCost_;
  double obstacleCost_;

  rclcpp::Node::SharedPtr node_;
  grid_map::GridMap map_;
};

}  // end of namespace planning

#endif
