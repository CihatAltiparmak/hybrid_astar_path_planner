#include <hybrid_astar_planner/path_smoother.hpp>

namespace planning {

PathSmoother::PathSmoother() {}

void PathSmoother::smoothPath(const nav_msgs::msg::Path::SharedPtr path) {
    int iteration_number = 500;

    for (int i = 0; i < iteration_number; i++) {
    }
}

void PathSmoother::obstacleTerm() {}

void PathSmoother::smoothingTerm() {}

}  // end of namespace planning