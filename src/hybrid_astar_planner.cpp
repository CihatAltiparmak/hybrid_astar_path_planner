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

#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <hybrid_astar_planner/path_smoother.hpp>

namespace planning
{

HybridAstarPlanner::HybridAstarPlanner(rclcpp::Node::SharedPtr node)
: node_(node)
{
  node_->declare_parameter("wheelbase", 2.0);
  node_->get_parameter("wheelbase", wheelbase_);

  node_->declare_parameter("direction_inputs", std::vector<double>({1.41 /*1.0*/}));
  node_->get_parameter("direction_inputs", directionInputs_);

  node_->declare_parameter(
    "steering_inputs",
    std::vector<double>(
      {-0.52, -0.45, -0.4, -0.35, -0.3, -0.25, -0.2, -0.15, -0.1, 0.0, 0.1, 0.15,
        0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.52}));

  node_->get_parameter("steering_inputs", steeringInputs_);

  node_->declare_parameter("time_limit", 100000);
  node_->get_parameter("time_limit", timeLimit_);

  node_->declare_parameter("max_iteration_limit", 30000);
  node_->get_parameter("max_iteration_limit", maxIterationNumber_);

  node_->declare_parameter("goal_tolerance", 0.5);
  node_->get_parameter("goal_tolerance", goalTolerance_);

  node_->declare_parameter("steering_cost", 0.5);
  node_->get_parameter("steering_cost", steeringCost_);

  node_->declare_parameter("heading_change_cost", 0.1);
  node_->get_parameter("heading_change_cost", headingChangeCost_);

  node_->declare_parameter("obstacle_cost", 0.1);
  node_->get_parameter("obstacle_cost", obstacleCost_);

  turningRadius_ = wheelbase_ / std::tan(0.52);
}

std::vector<std::shared_ptr<Node>> HybridAstarPlanner::plan(
  std::shared_ptr<Node> start_node_ptr, std::shared_ptr<Node> target_node_ptr)
{
  if (!isInsideOfMap(start_node_ptr)) {
    RCLCPP_ERROR(this->node_->get_logger(), "PLAN FAIL: start node not inside map");
    return {};
  }

  start_node_ptr->h_cost = heruisticCost(start_node_ptr, target_node_ptr);
  start_node_ptr->f_cost = start_node_ptr->h_cost;

  std::vector<std::shared_ptr<Node>> nodes;
  std::priority_queue<std::shared_ptr<Node>> open_list;
  std::vector<std::shared_ptr<Node>> cell_table(map_.getSize()(0) * map_.getSize()(1), nullptr);

  open_list.push(start_node_ptr);
  std::shared_ptr<Node> closest_node_ptr = start_node_ptr;
  updateExistingNodeInCell(start_node_ptr, cell_table);

  RCLCPP_DEBUG(this->node_->get_logger(), "PLAN START");

  int iteration_number = 0;
  while (!open_list.empty()) {
    iteration_number++;

    std::shared_ptr<Node> node_ptr = open_list.top();
    open_list.pop();

    if (iteration_number > maxIterationNumber_) {
      RCLCPP_DEBUG(this->node_->get_logger(), "PLAN TIMEOUT EXCEEDED : %d", iteration_number);
      break;
    }

    if (isGoalReached(node_ptr, target_node_ptr)) {
      RCLCPP_DEBUG(this->node_->get_logger(), "PLAN CREATED : %d", iteration_number);
      closest_node_ptr = node_ptr;
      break;
    }

    if (isClosed(node_ptr, cell_table)) {
      continue;
    }

    if (node_ptr->h_cost < closest_node_ptr->h_cost) {
      closest_node_ptr = node_ptr;
    }

    // nodes.push_back(node_ptr);
    addToClosedList(node_ptr, cell_table);
    updateNeigbour(node_ptr, target_node_ptr, nodes, open_list, cell_table);
  }

  std::shared_ptr<Node> path_current_node = closest_node_ptr;
  std::vector<std::shared_ptr<Node>> path;
  while (path_current_node != nullptr) {
    path.push_back(path_current_node);
    path_current_node = path_current_node->parent;
  }
  std::reverse(path.begin(), path.end());

  RCLCPP_DEBUG(this->node_->get_logger(), "Hybrid Path Size : %d", static_cast<int>(path.size()));
  RCLCPP_DEBUG(this->node_->get_logger(), "Closest Node Distance : %f", closest_node_ptr->h_cost);

  return path;
}

void HybridAstarPlanner::updateNeigbour(
  std::shared_ptr<Node> node_ptr, std::shared_ptr<Node> target_node_ptr,
  std::vector<std::shared_ptr<Node>> & nodes,
  std::priority_queue<std::shared_ptr<Node>> & open_list,
  std::vector<std::shared_ptr<Node>> & cell_table)
{
  for (const double & steering : steeringInputs_) {
    for (const double & direction : directionInputs_) {
      std::shared_ptr<Node> new_node_ptr = std::make_shared<Node>(
        node_ptr->getNextNode(steering, direction, wheelbase_));

      if (!isInsideOfMap(new_node_ptr) || !isPathValid(node_ptr, new_node_ptr)) {
        continue;
      }

      new_node_ptr->h_cost = heruisticCost(new_node_ptr, target_node_ptr);
      new_node_ptr->f_cost =
        new_node_ptr->h_cost +
        steeringCost_ * std::abs(steering) +
        headingChangeCost_ * std::abs(new_node_ptr->yaw - node_ptr->yaw);

      new_node_ptr->parent = node_ptr;
      open_list.push(new_node_ptr);

      updateExistingNodeInCell(new_node_ptr, cell_table);
    }
  }
}

void HybridAstarPlanner::updateExistingNodeInCell(
  std::shared_ptr<Node> new_node_ptr,
  std::vector<std::shared_ptr<Node>> & cell_table)
{
  grid_map::Index node_index = getIndexOfNode(new_node_ptr);
  int index = node_index.y() + node_index.x() * map_.getSize().x();

  if (cell_table[index] == nullptr) {
    cell_table[index] = new_node_ptr;
    return;
  }

  if (new_node_ptr->f_cost < cell_table[index]->f_cost) {
    cell_table[index] = new_node_ptr;
  }
}

double HybridAstarPlanner::heruisticCost(
  const std::shared_ptr<Node> & node_ptr,
  const std::shared_ptr<Node> & target_node_ptr)
{

  ompl::base::DubinsStateSpace dubinsPath(turningRadius_);
  State * dbStart = (State *)dubinsPath.allocState();
  State * dbEnd = (State *)dubinsPath.allocState();
  dbStart->setXY(node_ptr->x, node_ptr->y);
  dbStart->setYaw(node_ptr->yaw);
  dbEnd->setXY(target_node_ptr->x, target_node_ptr->y);
  dbEnd->setYaw(target_node_ptr->yaw);
  return dubinsPath.distance(dbStart, dbEnd);
}

void HybridAstarPlanner::addToClosedList(
  const std::shared_ptr<Node> & node_ptr,
  std::vector<std::shared_ptr<Node>> & cell_table)
{
  grid_map::Index node_index = getIndexOfNode(node_ptr);
  int index = node_index.y() + node_index.x() * map_.getSize().x();

  cell_table[index] = nullptr;
}

// https://gamedev.stackexchange.com/a/182143
// http://www.cse.yorku.ca/~amana/research/grid.pdf
bool HybridAstarPlanner::isPathValid(
  const std::shared_ptr<Node> & start_node_ptr,
  const std::shared_ptr<Node> & end_node_ptr)
{
  double resolution = map_.getResolution();

  double x0 = start_node_ptr->x / resolution;
  double y0 = start_node_ptr->y / resolution;

  double x1 = end_node_ptr->x / resolution;
  double y1 = end_node_ptr->y / resolution;

  double dx = x1 - x0;
  double dy = y1 - y0;

  double angle = std::atan2(dy, dx);
  double tDeltaX = std::abs(1.0 / std::cos(angle));
  double tDeltaY = std::abs(1.0 / std::sin(angle));

  double tMaxX, tMaxY;
  double stepX, stepY;

  if (dx < 0) {
    tMaxX = std::abs((x0 - std::ceil(x0 - 1)) / std::cos(angle));
    stepX = -1;
  } else {
    tMaxX = std::abs((std::ceil(x0) - x0) / std::cos(angle));
    stepX = 1;
  }

  if (dy < 0) {
    tMaxY = std::abs((y0 - std::ceil(y0 - 1)) / std::sin(angle));
    stepY = -1;
  } else {
    tMaxY = std::abs((std::ceil(y0) - y0) / std::sin(angle));
    stepY = 1;
  }

  double x = x0;
  double y = y0;

  double dist = std::abs(std::ceil(x1) - std::ceil(x0)) +
    std::abs(std::ceil(y1) - std::ceil(y0));

  for (int i = 0; i <= dist; i++) {
    grid_map::Position cell_pos(x * resolution, y * resolution);
    if (map_.atPosition("obstacle", cell_pos) > 0.0) {
      return false;
    }

    if (std::abs(tMaxX) < std::abs(tMaxY)) {
      tMaxX = tMaxX + tDeltaX;
      x = x + stepX;
    } else {
      tMaxY = tMaxY + tDeltaY;
      y = y + stepY;
    }
  }

  return true;
}

bool HybridAstarPlanner::isInsideOfMap(const std::shared_ptr<Node> & node_ptr)
{
  grid_map::Position node_position = getPositionOfNode(node_ptr);
  return map_.isInside(node_position);
}

// TODO @CihatAltiparmak : not implemented correctly. fix it
bool HybridAstarPlanner::isClosed(
  const std::shared_ptr<Node> & node_ptr,
  std::vector<std::shared_ptr<Node>> & cell_table)
{
  grid_map::Index node_index = getIndexOfNode(node_ptr);
  int index = node_index.y() + node_index.x() * map_.getSize().x();

  return node_ptr != cell_table[index];
}

bool HybridAstarPlanner::isGoalReached(
  const std::shared_ptr<Node> & current_node_ptr,
  const std::shared_ptr<Node> & target_node_ptr)
{
  double dx = current_node_ptr->x - target_node_ptr->x;
  double dy = current_node_ptr->y - target_node_ptr->y;
  double dt = current_node_ptr->yaw - target_node_ptr->yaw;

  double norm = std::sqrt(dx * dx + dy * dy + dt * dt);

  return norm < goalTolerance_;
}

grid_map::Position HybridAstarPlanner::getPositionOfNode(const std::shared_ptr<Node> & node_ptr)
{
  grid_map::Position node_position(node_ptr->x, node_ptr->y);
  return node_position;
}

grid_map::Index HybridAstarPlanner::getIndexOfNode(const std::shared_ptr<Node> & node_ptr)
{
  grid_map::Position node_position = getPositionOfNode(node_ptr);
  grid_map::Index node_index;
  map_.getIndex(node_position, node_index);
  return node_index;
}

std::vector<Vector2d> HybridAstarPlanner::convertPathToVector2dList(
  const std::vector<std::shared_ptr<Node>> & path)
{
  std::vector<Vector2d> generated_path;

  for (auto node : path) {
    Vector2d pointVect(node->x, node->y);
    generated_path.push_back(pointVect);
  }

  return generated_path;
}

planner_msgs::msg::Path HybridAstarPlanner::convertPlanToRosMsg(
  const std::vector<std::shared_ptr<Node>> & path)
{
  planner_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";

  for (auto node : path) {
    planner_msgs::msg::Point point;
    point.x = node->x;
    point.y = node->y;
    point.yaw = node->yaw;

    path_msg.points.push_back(point);
  }

  return path_msg;
}

}  // end of namespace planning
