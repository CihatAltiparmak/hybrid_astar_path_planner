#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <visualization_msgs/msg/marker.hpp>

grid_map::GridMap create_map() {
    grid_map::GridMap map =
        grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(10.0, 10.0), 1.0,
                    grid_map::Position(0.0, 0.0));

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);
        map.at("x", *it) = pos.x();
        map.at("y", *it) = pos.y();
        map.at("z", *it) = 0.0;
        map.at("obstacle", *it) = 0.0;

        std::cout << "elma: " << pos.x() << " | " << pos.y() << std::endl;
    }

    return map;
}

void ray_casting(planning::Node start_node, planning::Node end_node,
                 std::vector<std::pair<double, double>>& all_nodes) {
    double x = start_node.x;  // std::floor(start_node.x);
    double y = start_node.y;  // std::floor(start_node.y);

    double dx = end_node.x - start_node.x;
    double dy = end_node.y - start_node.y;

    int stepX = dx < 0 ? -1 : 1;
    int stepY = dy < 0 ? -1 : 1;

    double xOffset = end_node.x > start_node.x
                         ? std::ceil(start_node.x) - start_node.x
                         : start_node.x - std::floor(start_node.x);

    double yOffset = end_node.y > start_node.y
                         ? std::ceil(start_node.y) - start_node.y
                         : start_node.y - std::floor(start_node.y);

    double angle = std::atan2(dy, dx);

    double tMaxX = xOffset / std::cos(angle);
    double tMaxY = yOffset / std::sin(angle);

    double tDeltaX = 1.0 / std::cos(angle);
    double tDeltaY = 1.0 / std::sin(angle);

    double manhattanDist =
        std::abs(std::floor(end_node.x) - std::floor(start_node.x)) +
        std::abs(std::floor(end_node.y) - std::floor(start_node.y));

    for (int i = 0; i <= manhattanDist; i++) {
        all_nodes.push_back({x, y});
        std::cout << x << " | " << y << std::endl;
        if (tMaxX < tMaxY) {
            tMaxX += tDeltaX;
            x += stepX;
        } else {
            tMaxY += tDeltaY;
            y += stepY;
        }
    }
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
    start_node.x = -0.5;
    start_node.y = -0.5;
    start_node.yaw = 0.785398;

    planning::Node end_node = planning::Node();
    end_node.x = -2;
    end_node.y = -0.5;
    end_node.yaw = 0.785398;

    std::vector<std::pair<double, double>> all_nodes;
    ray_casting(start_node, end_node, all_nodes);

    std::cout << "neigbours start" << std::endl;
    for (auto node : all_nodes) {
        grid_map::Index n_indx;
        hybrid_astar.map.getIndex(grid_map::Position(node.first, node.second),
                                  n_indx);

        hybrid_astar.map.at("z", n_indx) = -2.0;
        hybrid_astar.map.at("obstacle", n_indx) = -2.0;
    }

    all_nodes = {{start_node.x, start_node.y}, {end_node.x, end_node.y}};
    for (auto node : all_nodes) {
        geometry_msgs::msg::Point node_point;
        node_point.x = node.first;
        node_point.y = node.second;
        node_point.z = 0.0;
        viz_msg.points.push_back(node_point);

        // grid_map::Index n_indx;
        // hybrid_astar.map.getIndex(grid_map::Position(node.first,
        // node.second), n_indx);
        //
        // hybrid_astar.map.at("z", n_indx) = 2.0;
        // hybrid_astar.map.at("obstacle", n_indx) = 2.0;
    }

    std::cout << "neigbours end: " << viz_msg.points.size() << std::endl;

    viz_msg.header.frame_id = "map";
    viz_msg.ns = "test_update_neigbour";
    viz_msg.action = visualization_msgs::msg::Marker::ADD;
    viz_msg.id = 1;
    viz_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    viz_msg.scale.x = 0.005;
    // viz_msg.scale.y = 0.5;
    // viz_msg.scale.z = 0.5;
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