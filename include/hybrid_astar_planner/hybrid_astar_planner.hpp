#ifndef __HYBRID_ASTAR_PLANNER_HPP__
#define __HYBRID_ASTAR_PLANNER_HPP__

#include <cmath>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace planning {

typedef struct Node {
    double x;
    double y;
    double yaw;
    double h_cost;
    double f_cost;
    Node *parent;

    Node() : Node(0.0, 0.0) {}
    Node(const double &x, const double &y) : x(x), y(y), yaw(0.0) {}

    Node getNextNode(const double &steering, const double &velocity,
                     const double &wheelbase, const double &dt) const {
        Node next_node;
        // std::cout << "getnextnode: cos" << velocity * std::cos(yaw) * dt <<
        // std::endl; std::cout << "getnextnode: sin" << velocity *
        // std::sin(yaw) * dt << std::endl;
        double delta_yaw =
            normalize_yaw(velocity * dt * std::tan(steering) / wheelbase);
        next_node.x = x + velocity * std::cos(yaw + delta_yaw) * dt;
        next_node.y = y + velocity * std::sin(yaw + delta_yaw) * dt;
        next_node.yaw = normalize_yaw(yaw + delta_yaw);
        return next_node;
    }

    double normalize_yaw(const double &yaw) const {
        double v = std::fmod(yaw, 2 * M_PI);

        if (v < -M_PI) {
            v += 2.0 * M_PI;
        } else if (v > M_PI) {
            v -= 2.0 * M_PI;
        }

        return v;
    }

} Node;

inline bool operator<(const std::shared_ptr<Node> &n1,
                      const std::shared_ptr<Node> &n2) {
    return n1->f_cost < n2->f_cost;
}

class HybridAstarPlanner {
   public:
    HybridAstarPlanner();
    std::vector<std::shared_ptr<Node> > plan(Node &, Node &);
    void updateNeigbour(const Node &, std::vector<std::shared_ptr<Node> > &,
                        std::priority_queue<std::shared_ptr<Node> > &,
                        std::vector<bool> &);
    // virtual bool isCollision() = 0;
    bool isPathValid(const Node &, const Node &);
    bool isInsideOfMap(const Node &);
    bool isClosed(const Node &, std::vector<bool> &);
    bool isInsideOfSameCell(const Node &, const Node &);
    grid_map::Index getIndexOfNode(const Node &);
    grid_map::Position getPositionOfNode(const Node &);
    // void pubMap();
    grid_map::GridMap map;

   private:
    std::vector<double> steeringInputs;
    std::vector<double> velocityInputs;
    double wheelbase;
    double dt;
};

}  // end of namespace planning

#endif