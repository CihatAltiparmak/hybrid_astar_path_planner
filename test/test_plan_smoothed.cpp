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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "test_utils/utils.hpp"

planning::HybridAstarPlanner hybrid_astar;

planning::Node start_node = planning::Node(-0.049 * 20, -0.049 * 20, 0.785398);
planning::Node end_node = planning::Node(0.4 * 20.0, 0.4 * 20.0, 0.785398);

rclcpp::Node::SharedPtr node;
planning::PathSmoother smoother;

visualization_msgs::msg::Marker unsmoothed_viz_msg;
visualization_msgs::msg::Marker smoothed_viz_msg;

grid_map::GridMap create_map() {
    grid_map::GridMap map =
        grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
    map.setFrameId("map");
    // map.setGeometry(grid_map::Length(1.0, 1.0), 0.05,
    //                 grid_map::Position(0.0, 0.0));

    map.setGeometry(grid_map::Length(20.0, 20.0), 1.0,
                    grid_map::Position(0.0, 0.0));

    clearGridMap(map);
    addObstaclesToGridMap(map);

    return map;
}

void do_plan() {
    std::cout << "YESSS DOING IT" << std::endl;
    std::vector<std::shared_ptr<planning::Node> > all_nodes;
    smoothed_viz_msg.points.clear();
    unsmoothed_viz_msg.points.clear();

    hybrid_astar.map_ = create_map();
    smoother.doSettingsWithMap(hybrid_astar.map_);

    auto unsmoothed_path = hybrid_astar.convertPathToVector2dList(
        hybrid_astar.plan(std::make_shared<planning::Node>(start_node),
                          std::make_shared<planning::Node>(end_node)));

    auto smoothed_path = smoother.smoothPath(unsmoothed_path);

    for (int i = 0; i < (int)unsmoothed_path.size(); i++) {
        geometry_msgs::msg::Point node_point;
        node_point.x = unsmoothed_path[i][0];
        node_point.y = unsmoothed_path[i][1];
        node_point.z = 0.0;
        unsmoothed_viz_msg.points.push_back(node_point);

        node_point.x = smoothed_path[i][0];
        node_point.y = smoothed_path[i][1];
        node_point.z = 0.0;
        smoothed_viz_msg.points.push_back(node_point);
    }
}

void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::cout << "initial_pose sub received msg" << std::endl;
    start_node.x = msg->pose.pose.position.x;
    start_node.y = msg->pose.pose.position.y;

    do_plan();
}

void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::cout << "goal_pose sub received msg" << std::endl;
    end_node.x = msg->pose.position.x;
    end_node.y = msg->pose.position.y;

    do_plan();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("hybrid_astar_node");

    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "/astar_grid_map", rclcpp::QoS(1).transient_local());

    auto unsmoothed_path_pub =
        node->create_publisher<visualization_msgs::msg::Marker>(
            "/unsmoothed_path", rclcpp::QoS(1).transient_local());

    auto smoothed_path_pub =
        node->create_publisher<visualization_msgs::msg::Marker>(
            "/smoothed_path", rclcpp::QoS(1).transient_local());

    auto initial_pose_sub = node->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
                                                       initial_pose_callback);

    auto goal_pose_sub =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, goal_pose_callback);

    hybrid_astar = planning::HybridAstarPlanner(node);
    smoother = planning::PathSmoother(node);

    unsmoothed_viz_msg.header.frame_id = "map";
    unsmoothed_viz_msg.ns = "test_update_neigbour";
    unsmoothed_viz_msg.action = visualization_msgs::msg::Marker::ADD;
    unsmoothed_viz_msg.id = 1;
    unsmoothed_viz_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    unsmoothed_viz_msg.scale.x = 0.005 * 20;
    unsmoothed_viz_msg.scale.y = 0.005 * 20;
    unsmoothed_viz_msg.scale.z = 0.005 * 20;
    unsmoothed_viz_msg.color.b = 1.0;
    unsmoothed_viz_msg.color.a = 1.0;

    smoothed_viz_msg.header.frame_id = "map";
    smoothed_viz_msg.ns = "el_planno_americano";
    smoothed_viz_msg.action = visualization_msgs::msg::Marker::ADD;
    smoothed_viz_msg.id = 2;
    smoothed_viz_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    smoothed_viz_msg.scale.x = 0.005 * 20;
    smoothed_viz_msg.scale.y = 0.005 * 20;
    smoothed_viz_msg.scale.z = 0.005 * 20;
    smoothed_viz_msg.color.b = 0.0;
    smoothed_viz_msg.color.a = 1.0;

    do_plan();

    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
            grid_map::GridMapRosConverter::toMessage(hybrid_astar.map_);

        map_pub->publish(std::move(msg));
        unsmoothed_path_pub->publish(unsmoothed_viz_msg);
        smoothed_path_pub->publish(smoothed_viz_msg);

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }
}