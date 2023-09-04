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
#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace planning {

class HybridAstarPlanner {
   public:
    HybridAstarPlanner();
    HybridAstarPlanner(rclcpp::Node::SharedPtr &);
    std::vector<std::shared_ptr<Node> > plan(std::shared_ptr<Node>,
                                             std::shared_ptr<Node>);
    void updateNeigbour(std::shared_ptr<Node>, std::shared_ptr<Node>,
                        std::vector<std::shared_ptr<Node> > &,
                        std::priority_queue<std::shared_ptr<Node> > &,
                        std::vector<bool> &);
    void addToClosedList(const Node &, std::vector<bool> &);
    double heruisticCost(const Node &, const Node &);
    // virtual bool isCollision() = 0;
    bool isPathValid(const Node &, const Node &);
    bool isInsideOfMap(const Node &);
    bool isClosed(const Node &, std::vector<bool> &);
    bool isInsideOfSameCell(const Node &, const Node &);
    grid_map::Index getIndexOfNode(const Node &);
    grid_map::Position getPositionOfNode(const Node &);
    // void pubMap();
    std::vector<Vector2d> convertPathToVector2dList(
        const std::vector<std::shared_ptr<Node> > &);

    grid_map::GridMap map_;

   private:
    std::vector<double> steeringInputs_;
    std::vector<double> velocityInputs_;
    double wheelbase_;
    double dt_;
    int timeLimit_;

    double steeringCost_;
    double headingChangeCost_;
    double obstacleCost_;
};

}  // end of namespace planning

#endif