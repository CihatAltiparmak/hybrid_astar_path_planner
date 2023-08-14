#include <hybrid_astar_planner/hybrid_astar_planner.hpp>

namespace planning {

HybridAstarPlanner::HybridAstarPlanner() {
    wheelbase = 1.0;  // 0.05;  // 2.0;
    dt = 0.1;
    velocityInputs = {10};  //{0.5};
    // steeringInputs = {-0.34, 0.0, 0.34};
    steeringInputs = {-0.34, -0.17, 0.0, 0.17, 0.34};
    // steeringInputs = {-0.78, 0.0, 0.78};
}

std::vector<std::shared_ptr<Node> > HybridAstarPlanner::plan(
    std::shared_ptr<Node> start_node, std::shared_ptr<Node> target_node) {
    std::vector<std::shared_ptr<Node> > nodes;
    std::priority_queue<std::shared_ptr<Node> > open_list;
    std::vector<bool> closed_list(map.getSize()(0) * map.getSize()(1), false);

    open_list.push(start_node);

    while (!open_list.empty()) {
        std::shared_ptr<Node> node = open_list.top();
        open_list.pop();

        if (isInsideOfSameCell(*node, *target_node)) {
            std::cout << "PLAN CREATED" << std::endl;

            std::shared_ptr<Node> path_current_node = node;
            std::vector<std::shared_ptr<Node> > path;
            while (path_current_node != nullptr) {
                path.push_back(path_current_node);
                path_current_node = path_current_node->parent;
            }

            return path;
        }

        addToClosedList(*node, closed_list);
        updateNeigbour(node, target_node, nodes, open_list, closed_list);
    }

    std::cout << "PLAN FAIL" << std::endl;
    return nodes;
}

void HybridAstarPlanner::updateNeigbour(
    std::shared_ptr<Node> node, std::shared_ptr<Node> target_node,
    std::vector<std::shared_ptr<Node> >& nodes,
    std::priority_queue<std::shared_ptr<Node> >& open_list,
    std::vector<bool>& closed_list) {
    for (const double& steering : steeringInputs) {
        for (const double& velocity : velocityInputs) {
            Node new_node =
                node->getNextNode(steering, velocity, wheelbase, dt);

            // TODO @CihatAltiparmak : will be addressed this issue again
            if (isInsideOfSameCell(*node, new_node)) {
                new_node = new_node.getNextNode(steering, velocity / 2.0,
                                                wheelbase, dt);
            }

            if (isInsideOfMap(new_node) && isPathValid(*node, new_node) &&
                !isClosed(new_node, closed_list)) {
                new_node.f_cost = heruisticCost(new_node, *target_node) +
                                  0.1 * std::abs(steering);
                new_node.parent = node;
                std::shared_ptr<Node> new_node_ptr =
                    std::make_shared<Node>(new_node);
                nodes.push_back(new_node_ptr);
                open_list.push(new_node_ptr);
            }
        }
    }
}

double HybridAstarPlanner::heruisticCost(const Node& node,
                                         const Node& target_node) {
    double dx = node.x - target_node.x;
    double dy = node.y - target_node.y;

    return std::sqrt(dx * dx + dy * dy);
}

void HybridAstarPlanner::addToClosedList(const Node& node,
                                         std::vector<bool>& closed_list) {
    grid_map::Index node_index = getIndexOfNode(node);
    int index = node_index.y() + node_index.x() * map.getSize().x();

    closed_list[index] = true;
}

// https://gamedev.stackexchange.com/a/182143
// http://www.cse.yorku.ca/~amana/research/grid.pdf
bool HybridAstarPlanner::isPathValid(const Node& start_node,
                                     const Node& end_node) {
    double resolution = map.getResolution();

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
        grid_map::Position cell_pos(x * resolution, y * resolution);
        if (map.atPosition("obstacle", cell_pos) > 0.0) {
            return false;
        }

        if (std::abs(tMaxX) < std::abs(tMaxY)) {
            tMaxX = tMaxX + tDeltaX;
            x = x + stepX;
        } else {
            tMaxY = tMaxY + tDeltaY;
            y = y + stepY;
        }
    }

    return true;
}

bool HybridAstarPlanner::isInsideOfMap(const Node& node) {
    grid_map::Position node_position = getPositionOfNode(node);
    return map.isInside(node_position);
}

// TODO @CihatAltiparmak : not implemented correctly. fix it
bool HybridAstarPlanner::isClosed(const Node& node,
                                  std::vector<bool>& closed_list) {
    grid_map::Index node_index = getIndexOfNode(node);
    int index = node_index.y() + node_index.x() * map.getSize().x();

    return closed_list[index];
}

bool HybridAstarPlanner::isInsideOfSameCell(const Node& n1, const Node& n2) {
    grid_map::Index n1_index = getIndexOfNode(n1);
    grid_map::Index n2_index = getIndexOfNode(n2);

    return (n1_index.x() == n2_index.x()) && (n1_index.y() == n2_index.y());
}

grid_map::Position HybridAstarPlanner::getPositionOfNode(const Node& node) {
    grid_map::Position node_position(node.x, node.y);
    return node_position;
}

grid_map::Index HybridAstarPlanner::getIndexOfNode(const Node& node) {
    grid_map::Position node_position = getPositionOfNode(node);
    grid_map::Index node_index;
    map.getIndex(node_position, node_index);
    return node_index;
}

}  // end of namespace planning