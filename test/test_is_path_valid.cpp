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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hybrid_astar_planner/utils/node.hpp>
#include <nav_msgs/msg/path.hpp>

#include "test_utils/utils.hpp"

class TestIsPathValid : public rclcpp::Node
{
public:
  TestIsPathValid();
  void initialize();
  void loop();
  grid_map::GridMap create_map();
  void doTraverse();
  void ray_casting(
    planning::Node, planning::Node,
    std::vector<std::pair<double, double>> &);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);

private:
  planning::Node start_node;
  planning::Node end_node;
  grid_map::GridMap map_;
  nav_msgs::msg::Path line_viz_msg_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
  SharedPtr initialPoseSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goalPoseSub;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr mapPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lineVizPub_;
};

TestIsPathValid::TestIsPathValid()
: Node("test_plan_smoothed_node") {}

void TestIsPathValid::initialize()
{
  start_node = planning::Node(4.0, 5);
  end_node = planning::Node(-4.0, -4.0);

  doTraverse();

  mapPub = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "/astar_grid_map", rclcpp::QoS(1).transient_local());

  lineVizPub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/path_line", rclcpp::QoS(1).transient_local());

  initialPoseSub = this->create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(
      &TestIsPathValid::initialPoseCallback, this,
      std::placeholders::_1));

  goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(
      &TestIsPathValid::goalPoseCallback, this,
      std::placeholders::_1));
}

void TestIsPathValid::loop()
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
    grid_map::GridMapRosConverter::toMessage(map_);

  mapPub->publish(std::move(msg));
  lineVizPub_->publish(line_viz_msg_);
}

void TestIsPathValid::doTraverse()
{
  std::cout << "YESSS DOING IT" << std::endl;

  std::vector<std::pair<double, double>> all_nodes;
  map_ = create_map();
  ray_casting(start_node, end_node, all_nodes);

  for (auto node : all_nodes) {
    grid_map::Index n_indx;
    map_.getIndex(grid_map::Position(node.first, node.second), n_indx);

    map_.at("z", n_indx) = -2.0;
    map_.at("obstacle", n_indx) = -2.0;
  }

  all_nodes = {{start_node.x, start_node.y}, {end_node.x, end_node.y}};
  line_viz_msg_.header.frame_id = "map";
  line_viz_msg_.poses.clear();
  for (auto node : all_nodes) {
    geometry_msgs::msg::PoseStamped node_pose;
    node_pose.pose.position.x = node.first;
    node_pose.pose.position.y = node.second;
    line_viz_msg_.poses.push_back(node_pose);
  }
}

// https://gamedev.stackexchange.com/a/182143
// http://www.cse.yorku.ca/~amana/research/grid.pdf
void TestIsPathValid::ray_casting(
  planning::Node start_node, planning::Node end_node,
  std::vector<std::pair<double, double>> & all_nodes)
{
  double resolution = 2.0;
  double x0 = start_node.x / resolution;
  double y0 = start_node.y / resolution;

  double x1 = end_node.x / resolution;
  double y1 = end_node.y / resolution;

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
    all_nodes.push_back({x * resolution, y * resolution});

    if (std::abs(tMaxX) < std::abs(tMaxY)) {
      tMaxX = tMaxX + tDeltaX;
      x = x + stepX;
    } else {
      tMaxY = tMaxY + tDeltaY;
      y = y + stepY;
    }
  }
}

void TestIsPathValid::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << "initial_pose sub received msg" << std::endl;
  start_node.x = msg->pose.pose.position.x;
  start_node.y = msg->pose.pose.position.y;

  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);
  start_node.yaw = y;

  doTraverse();
}

void TestIsPathValid::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::cout << "goal_pose sub received msg" << std::endl;
  end_node.x = msg->pose.position.x;
  end_node.y = msg->pose.position.y;

  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);
  end_node.yaw = y;

  doTraverse();
}

grid_map::GridMap TestIsPathValid::create_map()
{
  grid_map::GridMap map =
    grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
  map.setFrameId("map");
  // map.setGeometry(grid_map::Length(1.0, 1.0), 0.05,
  //                 grid_map::Position(0.0, 0.0));

  map.setGeometry(
    grid_map::Length(20.0, 20.0), 2.0,
    grid_map::Position(0.0, 0.0));

  clearGridMap(map);

  return map;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestIsPathValid>();
  node->initialize();

  rclcpp::Rate rate(30.0);
  while (rclcpp::ok()) {
    node->loop();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
}
