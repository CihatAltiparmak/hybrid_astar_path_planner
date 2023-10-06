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
#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "test_utils/utils.hpp"

class TestPlanSmoothed : public rclcpp::Node
{
public:
  TestPlanSmoothed();
  void initialize();
  void loop();
  grid_map::GridMap create_map();
  void doPlan();
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);

private:
  planning::Node start_node;
  planning::Node end_node;

  planning::HybridAstarPlanner hybrid_astar;
  planning::PathSmoother smoother;

  planner_msgs::msg::Path smoothed_path_msg;
  planner_msgs::msg::Path unsmoothed_path_msg;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
  SharedPtr initialPoseSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goalPoseSub;

  rclcpp::Publisher<planner_msgs::msg::Path>::SharedPtr smoothedPathPub;
  rclcpp::Publisher<planner_msgs::msg::Path>::SharedPtr unsmoothedPathPub;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr mapPub;
};

TestPlanSmoothed::TestPlanSmoothed()
: Node("test_plan_smoothed_node") {}

void TestPlanSmoothed::initialize()
{
  start_node = planning::Node(-0.049 * 20 /** 20*/, -0.049 * 20 /** 20*/, 0.785398);
  end_node = planning::Node(0.4 * 20 /** 20.0*/, 0.4 * 20 /** 20.0*/, 0.785398);

  hybrid_astar = planning::HybridAstarPlanner(shared_from_this());
  smoother = planning::PathSmoother(shared_from_this());

  doPlan();

  mapPub = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "/astar_grid_map", rclcpp::QoS(1).transient_local());

  unsmoothedPathPub = this->create_publisher<planner_msgs::msg::Path>(
    "/unsmoothed_path", rclcpp::QoS(1).transient_local());

  smoothedPathPub = this->create_publisher<planner_msgs::msg::Path>(
    "/smoothed_path", rclcpp::QoS(1).transient_local());

  initialPoseSub = this->create_subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(
      &TestPlanSmoothed::initialPoseCallback, this,
      std::placeholders::_1));

  goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(
      &TestPlanSmoothed::goalPoseCallback, this,
      std::placeholders::_1));
}

void TestPlanSmoothed::loop()
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
    grid_map::GridMapRosConverter::toMessage(hybrid_astar.getMap());

  mapPub->publish(std::move(msg));
  unsmoothedPathPub->publish(unsmoothed_path_msg);
  smoothedPathPub->publish(smoothed_path_msg);
}

void TestPlanSmoothed::doPlan()
{
  std::cout << "YESSS DOING IT" << std::endl;

  hybrid_astar.setMap(create_map());

  unsmoothed_path_msg =
    hybrid_astar.plan(
    std::make_shared<planning::Node>(start_node),
    std::make_shared<planning::Node>(end_node));

  smoother.doSettingsWithMap(hybrid_astar.getMap());
  smoothed_path_msg =
    smoother.smoothPath(unsmoothed_path_msg);
}

void TestPlanSmoothed::initialPoseCallback(
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

  doPlan();
}

void TestPlanSmoothed::goalPoseCallback(
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

  doPlan();
}

grid_map::GridMap TestPlanSmoothed::create_map()
{
  grid_map::GridMap map =
    grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
  map.setFrameId("map");

  map.setGeometry(
    grid_map::Length(80.0, 80.0), 1.0 /*1.0*/,
    grid_map::Position(0.0, 0.0));

  clearGridMap(map);
  addObstaclesToGridMap(map);

  return map;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestPlanSmoothed>();
  node->initialize();

  rclcpp::Rate rate(30.0);
  while (rclcpp::ok()) {
    node->loop();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
}
