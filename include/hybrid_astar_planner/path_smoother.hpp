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

#ifndef __PATH_SMOOTHER__
#define __PATH_SMOOTHER__

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <hybrid_astar_planner/kd_tree.hpp>
#include <hybrid_astar_planner/utils/vector.hpp>
#include <planner_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planning {

class PathSmoother {
   public:
    PathSmoother() = default;
    PathSmoother(rclcpp::Node::SharedPtr);
    planner_msgs::msg::Path smoothPath(const std::vector<Vector2d>&);
    Vector2d obstacleTerm(const Vector2d&);
    Vector2d smoothingTerm(const Vector2d&, const Vector2d&, const Vector2d&,
                           const Vector2d&, const Vector2d&);
    void doSettingsWithMap(const grid_map::GridMap&);

   private:
    int iterationNumber_;
    double alpha_;
    double Wobst_;
    double Wsmoothness_;
    double distMax_;
    grid_map::GridMap map_;
    KDTree kdTree_;
};

}  // end of namespace planning

#endif