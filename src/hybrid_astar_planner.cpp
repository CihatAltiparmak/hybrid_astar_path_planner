#include <hybrid_astar_planner/hybrid_astar_planner.hpp>

namespace planning {

HybridAstarPlanner::HybridAstarPlanner() {
    // grid_map::GridMap map({"x", "y", "z"});
    map = grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
    map.setFrameId("map");
    // map.setGeometry(grid_map::Length(1.2, 2.0), 0.03,
    // grid_map::Position(0.0, -0.1));
    map.setGeometry(grid_map::Length(1.0, 1.0), 0.05,
                    grid_map::Position(0.0, 0.0));

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);

        map.at("x", *it) = pos.x();
        map.at("y", *it) = pos.y();
        map.at("z", *it) = 0.0;
        map.at("obstacle", *it) = 0.0;
    }

    grid_map::Index start(18, 2);
    grid_map::Index end(2, 13);

    grid_map::Position pos_start;
    map.getPosition(start, pos_start);

    grid_map::Position pos_end;
    map.getPosition(end, pos_end);

    std::cout << "---------------------" << std::endl;
    std::cout << pos_start << std::endl;
    std::cout << "***********" << std::endl;
    std::cout << pos_end << std::endl;
    std::cout << "---------------------" << std::endl;

    for (grid_map::LineIterator it(map, start, end); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);

        map.at("z", *it) = 5.0;
        map.at("obstacle", *it) = 1.0;
    }

    auto start_node = Node(pos_start.x(), pos_start.y());
    auto target_node = Node(pos_end.x(), pos_end.y());

    isPathValid(start_node, target_node);
    plan(start_node, target_node);
}

std::vector<std::shared_ptr<Node> > HybridAstarPlanner::plan(
    Node &start_node, Node &target_node) {
    std::vector<std::shared_ptr<Node> > nodes = {
        std::make_shared<Node>(start_node),
        std::make_shared<Node>(target_node)};
    std::priority_queue<std::shared_ptr<Node> > open_list;

    open_list.push(std::make_shared<Node>(start_node));
    open_list.push(std::make_shared<Node>(target_node));
    return nodes;
}

bool HybridAstarPlanner::isPathValid(const Node &n_start,
                                     const Node &n_target) {
    const grid_map::Index &n_start_index = getIndexOfNode(n_start);
    const grid_map::Index &n_target_index = getIndexOfNode(n_target);

    for (grid_map::LineIterator cell_iterator(map, n_start_index,
                                              n_target_index);
         !cell_iterator.isPastEnd(); ++cell_iterator) {
        grid_map::Position pos;
        map.getPosition(*cell_iterator, pos);

        if (map.at("obstacle", *cell_iterator) == 1.0) {
            return false;
        }
    }

    return true;
}

grid_map::Index HybridAstarPlanner::getIndexOfNode(const Node &node) {
    grid_map::Position node_position(node.x, node.y);
    grid_map::Index node_index;
    map.getIndex(node_position, node_index);
    return node_index;
}

}  // end of namespace planning

int main(int argc, char **argv) {
    planning::Node n1;
    planning::Node n2(1.0, 2.0);

    planning::operator<(std::make_shared<planning::Node>(n1),
                        std::make_shared<planning::Node>(n2));

    rclcpp::init(argc, argv);

    auto hybrid_astar = planning::HybridAstarPlanner();
    rclcpp::Node::SharedPtr node =
        std::make_shared<rclcpp::Node>("hybrid_astar_node");

    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "/astar_grid_map", rclcpp::QoS(1).transient_local());

    rclcpp::Rate rate(30.0);

    while (rclcpp::ok()) {
        std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
            grid_map::GridMapRosConverter::toMessage(hybrid_astar.map);
        map_pub->publish(std::move(msg));
        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }
}