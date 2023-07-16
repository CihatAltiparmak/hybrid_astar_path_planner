#ifndef __HYBRID_ASTAR_PLANNER_HPP__
#define __HYBRID_ASTAR_PLANNER_HPP__

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

    Node() {}
    Node(const double &x, const double &y) : x(x), y(y) {}
    bool operator<(const Node &node) const {
        return this->f_cost < node.f_cost;
    }

} Node;

bool operator<(const std::shared_ptr<Node> &n1,
               const std::shared_ptr<Node> &n2) {
    std::cout << "bumm: " << n1->x << std::endl;
    std::cout << "bumm: " << n2->x << std::endl;
    return n1->f_cost < n2->f_cost;
}

class HybridAstarPlanner {
   public:
    HybridAstarPlanner();
    std::vector<std::shared_ptr<Node> > plan(Node &, Node &);
    // virtual void updateNeigbour() = 0;
    // virtual bool isCollision() = 0;
    bool isPathValid(const Node &, const Node &);
    grid_map::Index getIndexOfNode(const Node &);
    // void pubMap();
    grid_map::GridMap map;

   private:
    std::vector<double> steeringInputs;
    std::vector<double> velocityInputs;
};

}  // end of namespace planning

#endif