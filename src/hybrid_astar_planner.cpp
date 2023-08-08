#include <hybrid_astar_planner/hybrid_astar_planner.hpp>

namespace planning {

HybridAstarPlanner::HybridAstarPlanner() {
    wheelbase = 0.05;  // 2.0;
    dt = 0.1;
    velocityInputs = {0.5};
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

        if (isClosed(*node, closed_list)) {
            continue;
        }

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
    return {};
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

bool HybridAstarPlanner::isPathValid(const Node& n_start,
                                     const Node& n_target) {
    const grid_map::Index& n_start_index = getIndexOfNode(n_start);
    const grid_map::Index& n_target_index = getIndexOfNode(n_target);

    for (grid_map::LineIterator cell_iterator(map, n_start_index,
                                              n_target_index);
         !cell_iterator.isPastEnd(); ++cell_iterator) {
        grid_map::Position pos;
        map.getPosition(*cell_iterator, pos);

        if (map.at("obstacle", *cell_iterator) > 0.0) {
            return false;
        }
    }

    return true;
}

bool HybridAstarPlanner::isInsideOfMap(const Node& node) {
    grid_map::Position node_position = getPositionOfNode(node);
    if (!map.isInside(node_position)) {
        std::cout << "not inside map" << std::endl;
    }
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