#include <hybrid_astar_planner/path_smoother.hpp>

namespace planning {

PathSmoother::PathSmoother(const grid_map::GridMap& map) : map_(map) {
    std::vector<KDPoint> obstacles;

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        if (map_.at("obstacle", *it) > 0.0) {
            grid_map::Position obstacle_pos;
            map_.getPosition(*it, obstacle_pos);
            obstacles.push_back({obstacle_pos.x(), obstacle_pos.y()});
        }
    }
    kdTree_ = KDTree(obstacles, 2);
}

std::vector<Vector2d> PathSmoother::smoothPath(
    const std::vector<Vector2d>& path) {
    int iteration_number = 5000;
    double alpha = 0.0001;
    double Wobst = 0.2;

    std::vector<Vector2d> new_path = path;

    for (int i = 0; i < iteration_number; i++) {
        for (int i = 1; i < static_cast<int>(new_path.size()) - 1; i++) {
            Vector2d correction = obstacleTerm(new_path[i]);
            new_path[i] -= alpha * Wobst * correction;
        }
    }

    return new_path;
}

Vector2d PathSmoother::obstacleTerm(const Vector2d& pointVect) {
    Vector2d nearestObstVect =
        Vector2d(kdTree_.getNearestPoint({pointVect[0], pointVect[1]}));
    Vector2d obstVector = pointVect - nearestObstVect;
    double dist = obstVector.norm2();
    double distMax = 5;

    if (dist < distMax) {
        return (2 * (dist - distMax) * obstVector) / dist;
    }

    return Vector2d(0.0, 0.0);
}

void PathSmoother::smoothingTerm() {}

}  // end of namespace planning