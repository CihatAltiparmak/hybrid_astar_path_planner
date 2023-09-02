#ifndef __PATH_SMOOTHER__
#define __PATH_SMOOTHER__

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <hybrid_astar_planner/kd_tree.hpp>
#include <hybrid_astar_planner/utils/vector.hpp>
#include <planner_msgs/msg/path.hpp>

namespace planning {

class PathSmoother {
   public:
    PathSmoother(const grid_map::GridMap&);
    std::vector<Vector2d> smoothPath(const std::vector<Vector2d>&);
    Vector2d obstacleTerm(const Vector2d&);
    void smoothingTerm();

   private:
    grid_map::GridMap map_;
    KDTree kdTree_;
};

}  // end of namespace planning

#endif