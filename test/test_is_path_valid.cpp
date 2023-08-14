#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <hybrid_astar_planner/hybrid_astar_planner.hpp>
#include <visualization_msgs/msg/marker.hpp>

auto hybrid_astar = planning::HybridAstarPlanner();

planning::Node start_node = planning::Node(4.0, 5);
planning::Node end_node = planning::Node(-4.0, -4.0);

visualization_msgs::msg::Marker viz_msg;

void clearMap(grid_map::GridMap& map) {
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);
        map.at("z", *it) = 0.0;
        map.at("obstacle", *it) = 0.0;
    }
}

grid_map::GridMap create_map() {
    grid_map::GridMap map =
        grid_map::GridMap({"x", "y", "z", "obstacle", "closed"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(20.0, 20.0), 2.0,
                    grid_map::Position(0.0, 0.0));

    clearMap(map);

    return map;
}

// https://gamedev.stackexchange.com/a/182143
// http://www.cse.yorku.ca/~amana/research/grid.pdf
void ray_casting(planning::Node start_node, planning::Node end_node,
                 std::vector<std::pair<double, double>>& all_nodes) {
    double resolution = 2.0;
    double x0 = start_node.x / resolution;
    double y0 = start_node.y / resolution;

    double x1 = end_node.x / resolution;
    double y1 = end_node.y / resolution;

    double dx = x1 - x0;
    double dy = y1 - y0;

    double angle = std::atan2(dy, dx);
    double tDeltaX = std::abs(1.0 / std::cos(angle));
    double tDeltaY = std::abs(1.0 / std::sin(angle));

    double tMaxX, tMaxY;
    double stepX, stepY;

    if (dx < 0) {
        tMaxX = std::abs((x0 - std::ceil(x0 - 1)) / std::cos(angle));
        stepX = -1;
    } else {
        tMaxX = std::abs((std::ceil(x0) - x0) / std::cos(angle));
        stepX = 1;
    }

    if (dy < 0) {
        tMaxY = std::abs((y0 - std::ceil(y0 - 1)) / std::sin(angle));
        stepY = -1;
    } else {
        tMaxY = std::abs((std::ceil(y0) - y0) / std::sin(angle));
        stepY = 1;
    }

    double x = x0;
    double y = y0;

    double dist = std::abs(std::ceil(x1) - std::ceil(x0)) +
                  std::abs(std::ceil(y1) - std::ceil(y0));

    for (int i = 0; i <= dist; i++) {
        all_nodes.push_back({x * resolution, y * resolution});

        if (std::abs(tMaxX) < std::abs(tMaxY)) {
            tMaxX = tMaxX + tDeltaX;
            x = x + stepX;
        } else {
            tMaxY = tMaxY + tDeltaY;
            y = y + stepY;
        }
    }
}

void traverse_grid() {
    std::vector<std::pair<double, double>> all_nodes;
    viz_msg.points.clear();
    clearMap(hybrid_astar.map);
    ray_casting(start_node, end_node, all_nodes);

    std::cout << "neigbours start" << std::endl;
    for (auto node : all_nodes) {
        // geometry_msgs::msg::Point node_point;
        // node_point.x = node.first;
        // node_point.y = node.second;
        // node_point.z = 0.0;
        // viz_msg.points.push_back(node_point);

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
}

void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::cout << "initial_pose sub received msg" << std::endl;
    start_node.x = msg->pose.pose.position.x;
    start_node.y = msg->pose.pose.position.y;

    traverse_grid();
}

void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::cout << "goal_pose sub received msg" << std::endl;
    end_node.x = msg->pose.position.x;
    end_node.y = msg->pose.position.y;

    traverse_grid();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    hybrid_astar.map = create_map();

    rclcpp::Node::SharedPtr node =
        std::make_shared<rclcpp::Node>("hybrid_astar_node");

    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>(
        "/astar_grid_map", rclcpp::QoS(1).transient_local());

    auto points_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/points", rclcpp::QoS(1).transient_local());

    auto initial_pose_sub = node->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
                                                       initial_pose_callback);

    auto goal_pose_sub =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, goal_pose_callback);

    viz_msg.header.frame_id = "map";
    viz_msg.ns = "test_update_neigbour";
    viz_msg.action = visualization_msgs::msg::Marker::ADD;
    viz_msg.id = 1;
    viz_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;  // SPHERE_LIST;
    viz_msg.scale.x = 0.1;
    viz_msg.scale.y = 0.1;
    viz_msg.scale.z = 0.1;
    viz_msg.color.b = 1.0;
    viz_msg.color.a = 1.0;

    traverse_grid();

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