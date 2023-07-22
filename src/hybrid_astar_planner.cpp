#include <hybrid_astar_planner/hybrid_astar_planner.hpp>

namespace planning {

HybridAstarPlanner::HybridAstarPlanner() {
    wheelbase = 0.05; // 2.0;
    dt = 0.1;
    velocityInputs = {0.5};
    // steeringInputs = {-0.34, 0.0, 0.34};
    steeringInputs = {-0.78, 0.0, 0.78};
}

std::vector<std::shared_ptr<Node> > HybridAstarPlanner::plan(
    Node& start_node, Node& target_node) {
    std::vector<std::shared_ptr<Node> > nodes = {
        std::make_shared<Node>(start_node),
        std::make_shared<Node>(target_node)};
    std::priority_queue<std::shared_ptr<Node> > open_list;
    std::vector<bool> closed_list;

    open_list.push(std::make_shared<Node>(start_node));
    open_list.push(std::make_shared<Node>(target_node));
    return nodes;
}

void HybridAstarPlanner::updateNeigbour(
    const Node& node, std::vector<std::shared_ptr<Node> >& nodes,
    std::priority_queue<std::shared_ptr<Node> >& open_list,
    std::vector<bool>& closed_list) {
    for (const double& steering : steeringInputs) {
        for (const double& velocity : velocityInputs) {
            Node new_node = node.getNextNode(steering, velocity, wheelbase, dt);

            // TODO @CihatAltiparmak : will be addressed this issue again
            if (isInsideOfSameCell(node, new_node)) {
                new_node = new_node.getNextNode(steering, velocity / 2.0,
                                                wheelbase, dt);
            }

            if (isInsideOfMap(new_node) && isPathValid(node, new_node) &&
                !isClosed(new_node, closed_list)) {
                std::shared_ptr<Node> new_node_ptr =
                    std::make_shared<Node>(new_node);
                nodes.push_back(new_node_ptr);
                open_list.push(new_node_ptr);
            }
        }
    }
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
            std::cout << "path is not valid" << std::endl;
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
    // std::cout << node_index << std::endl;
    int index = node_index.y() + node_index.x() * map.getSize().x();

    if (closed_list[index]) {
        std::cout << "in closed list" << std::endl;
    }

    return closed_list[index];
}

bool HybridAstarPlanner::isInsideOfSameCell(const Node& n1, const Node& n2) {
    grid_map::Index n1_index = getIndexOfNode(n1);
    grid_map::Index n2_index = getIndexOfNode(n2);

    std::cout << "[isInsideOfSameCell] : START" << std::endl;
    std::cout << n1_index.x() << " | " << n1_index.y() << std::endl;
    std::cout << n2_index.x() << " | " << n2_index.y() << std::endl;
    if ((n1_index.x() == n2_index.x()) && (n1_index.y() == n2_index.y())) {
        std::cout << "IN SAME CELL" << std::endl;
    }
    std::cout << "[isInsideOfSameCell] : END" << std::endl;

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