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
    double Wsmoothness = 0.2;

    std::vector<Vector2d> new_path = path;

    for (int i = 0; i < iteration_number; i++) {
        Vector2d correction(0.0, 0.0);

        for (int i = 2; i < static_cast<int>(new_path.size()) - 2; i++) {
            correction =
                Wsmoothness * smoothingTerm(new_path[i - 2], new_path[i - 1],
                                            new_path[i], new_path[i + 1],
                                            new_path[i + 2]);
            new_path[i] -= alpha * correction;
        }

        for (int i = 1; i < static_cast<int>(new_path.size()) - 1; i++) {
            correction = Wobst * obstacleTerm(new_path[i]);
            new_path[i] -= alpha * correction;
        }
    }

    return new_path;
}

Vector2d PathSmoother::obstacleTerm(const Vector2d& pointVect) {
    Vector2d nearestObstVect =
        Vector2d(kdTree_.getNearestPoint({pointVect[0], pointVect[1]}));
    Vector2d obstVector = pointVect - nearestObstVect;
    double dist = obstVector.norm2();
    double distMax = 3;

    if (dist < distMax) {
        return (2 * (dist - distMax) * obstVector) / dist;
    }

    return Vector2d(0.0, 0.0);
}

Vector2d PathSmoother::smoothingTerm(const Vector2d& xim2, const Vector2d& xim1,
                                     const Vector2d& xi, const Vector2d& xip1,
                                     const Vector2d& xip2) {
    return 2 * (xip2 - 4 * xip1 + 6 * xi - 4 * xim1 + xim2);
}

}  // end of namespace planning