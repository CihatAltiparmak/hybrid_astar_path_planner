#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <visualization_msgs/msg/marker.hpp>

grid_map::GridMap create_map() {
    grid_map::GridMap map =
        grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
    map.setFrameId("map");
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

    grid_map::Index start(7, 7);
    grid_map::Index end(8, 8);

    grid_map::Position pos_start;
    map.getPosition(start, pos_start);

    grid_map::Position pos_end;
    map.getPosition(end, pos_end);

    for (grid_map::LineIterator it(map, start, end); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);

        map.at("z", *it) = 10.0;
        map.at("obstacle", *it) = 10.0;
    }

    return map;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto hybrid_astar = planning::HybridAstarPlanner();
    hybrid_astar.map = create_map();

    rclcpp::Node::SharedPtr node =
        std::make_shared<rclcpp::Node>("hybrid_astar_node");

    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "/astar_grid_map", rclcpp::QoS(1).transient_local());

    auto points_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/points", rclcpp::QoS(1).transient_local());

    visualization_msgs::msg::Marker viz_msg;
    planning::Node start_node = planning::Node();
    start_node.x = -0.049;
    start_node.y = -0.049;
    start_node.yaw = 0.785398;
    int cell_number =
        hybrid_astar.map.getSize()(0) * hybrid_astar.map.getSize()(1);
    std::vector<std::shared_ptr<planning::Node> > all_nodes;
    std::priority_queue<std::shared_ptr<planning::Node> > pq;
    std::vector<bool> visited(cell_number, false);

    all_nodes.push_back(std::make_shared<planning::Node>(start_node));

    int level_ii = 0;
    for (int level = 0; level < 6; level++) {
        int node_size_limit = all_nodes.size();
        for (; level_ii < node_size_limit; level_ii++) {
            auto node = all_nodes[level_ii];
            hybrid_astar.updateNeigbour(*node, all_nodes, pq, visited);
        }
    }

    std::cout << "neigbours start" << std::endl;
    for (auto node : all_nodes) {
        geometry_msgs::msg::Point node_point;
        node_point.x = node->x;
        node_point.y = node->y;
        node_point.z = 0.0;
        viz_msg.points.push_back(node_point);

        grid_map::Index n_indx = hybrid_astar.getIndexOfNode(*node);

        hybrid_astar.map.at("z", n_indx) = -2.0;
        hybrid_astar.map.at("obstacle", n_indx) = -2.0;
        std::cout << node->x << " | " << node->y << " | " << node->yaw
                  << std::endl;
    }
    std::cout << "neigbours end: " << viz_msg.points.size() << std::endl;

    viz_msg.header.frame_id = "map";
    viz_msg.ns = "test_update_neigbour";
    viz_msg.action = visualization_msgs::msg::Marker::ADD;
    viz_msg.id = 1;
    viz_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    viz_msg.scale.x = 0.005;
    viz_msg.scale.y = 0.005;
    viz_msg.scale.z = 0.005;
    viz_msg.color.b = 1.0;
    viz_msg.color.a = 1.0;

    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()) {
        std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
            grid_map::GridMapRosConverter::toMessage(hybrid_astar.map);

        map_pub->publish(std::move(msg));
        points_pub->publish(viz_msg);

        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }
}