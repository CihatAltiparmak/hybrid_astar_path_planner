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

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <memory>
#include <planner_msgs/msg/path.hpp>

#include "test_utils/utils.hpp"

class TestUpdateNeigbour : public rclcpp::Node
{
public:
  TestUpdateNeigbour();
  void initialize();
  void loop();
  grid_map::GridMap create_map();
  void doUpdateNeigbour();
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

private:
  planning::Node start_node;

  planning::HybridAstarPlanner hybrid_astar;

  planner_msgs::msg::Path all_path_msg;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
  SharedPtr initialPoseSub;

  rclcpp::Publisher<planner_msgs::msg::Path>::SharedPtr allPathPub;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr mapPub;
};

TestUpdateNeigbour::TestUpdateNeigbour()
: Node("test_plan_smoothed_node") {}

void TestUpdateNeigbour::initialize()
{
  start_node = planning::Node(-0.049 * 20, -0.049 * 20, 0.785398);

  hybrid_astar = planning::HybridAstarPlanner(shared_from_this());

  doUpdateNeigbour();

  mapPub = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "/astar_grid_map", rclcpp::QoS(1).transient_local());

  allPathPub = this->create_publisher<planner_msgs::msg::Path>(
    "/all_path", rclcpp::QoS(1).transient_local());

  initialPoseSub = this->create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(
      &TestUpdateNeigbour::initialPoseCallback, this,
      std::placeholders::_1));
}

void TestUpdateNeigbour::loop()
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
    grid_map::GridMapRosConverter::toMessage(hybrid_astar.map_);

  mapPub->publish(std::move(msg));
  allPathPub->publish(all_path_msg);
}

void TestUpdateNeigbour::doUpdateNeigbour()
{
  std::cout << "YESSS DOING TO Update Neigbour" << std::endl;

  hybrid_astar.map_ = create_map();

  int cell_number =
    hybrid_astar.map_.getSize()(0) * hybrid_astar.map_.getSize()(1);

  std::vector<std::shared_ptr<planning::Node>> all_nodes;
  std::priority_queue<std::shared_ptr<planning::Node>> pq;
  std::vector<std::shared_ptr<planning::Node>> visited(cell_number, nullptr);

  all_nodes.push_back(std::make_shared<planning::Node>(start_node));

  int level_ii = 0;
  for (int level = 0; level < 2; level++) {
    int node_size_limit = all_nodes.size();
    for (; level_ii < node_size_limit; level_ii++) {
      auto node_ptr = all_nodes[level_ii];
      hybrid_astar.updateNeigbour(
        node_ptr, node_ptr, all_nodes, pq,
        visited);
    }
  }

  all_path_msg = hybrid_astar.convertPlanToRosMsg(all_nodes);
}

void TestUpdateNeigbour::initialPoseCallback(
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

  doUpdateNeigbour();
}

grid_map::GridMap TestUpdateNeigbour::create_map()
{
  grid_map::GridMap map =
    grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
  map.setFrameId("map");
  // map.setGeometry(grid_map::Length(1.0, 1.0), 0.05,
  //                 grid_map::Position(0.0, 0.0));

  map.setGeometry(
    grid_map::Length(80.0, 80.0), 1.0,
    grid_map::Position(0.0, 0.0));

  clearGridMap(map);
  addObstaclesToGridMap(map);

  return map;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestUpdateNeigbour>();
  node->initialize();

  rclcpp::Rate rate(30.0);
  while (rclcpp::ok()) {
    node->loop();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
}
