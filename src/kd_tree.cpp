/*
 * MIT License
 *
 * Copyright (c) 2023 CihatAltiparmak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <hybrid_astar_planner/kd_tree.hpp>

namespace planning {

KDTree::KDTree(std::vector<KDPoint>& point_list, const int& dimension) {
    dimension_ = dimension;
    rootPtr_ = build(point_list, 0, static_cast<int>(point_list.size()) - 1, 0);
}

std::shared_ptr<KDNode> KDTree::build(std::vector<KDPoint>& point_list,
                                      const int& left_index,
                                      const int& right_index,
                                      const int& depth) const {
    if (point_list.empty() || left_index > right_index) {
        return nullptr;
    }

    auto sort_key_function = [depth](const KDPoint& point1,
                                     const KDPoint& point2) {
        return point1[depth] < point2[depth];
    };

    // TODO: add key function for builtin sort function
    std::sort(point_list.begin() + left_index, point_list.begin() + right_index,
              sort_key_function);

    const int& mid_index = (left_index + right_index) / 2;
    std::shared_ptr<KDNode> leftChildKDNodePtr =
        build(point_list, left_index, mid_index - 1, (depth + 1) % dimension_);
    std::shared_ptr<KDNode> rightChildKDNodePtr =
        build(point_list, mid_index + 1, right_index, (depth + 1) % dimension_);
    std::shared_ptr<KDNode> rootKDNode = std::make_shared<KDNode>(
        KDNode(point_list[mid_index], leftChildKDNodePtr, rightChildKDNodePtr));

    return rootKDNode;
}

void KDTree::insertPoint(KDPoint new_point) {
    insertPoint(new_point, rootPtr_, 0);
}

void KDTree::insertPoint(const KDPoint& new_point,
                         std::shared_ptr<KDNode>& node, const int& depth) {
    if (node == nullptr) {
        node = std::make_shared<KDNode>(new_point, nullptr, nullptr);
        return;
    }

    if (new_point[depth] < node->point[depth]) {
        insertPoint(new_point, node->leftChildKDNodePtr,
                    (depth + 1) % dimension_);
    } else {
        insertPoint(new_point, node->rightChildKDNodePtr,
                    (depth + 1) % dimension_);
    }
}

void KDTree::deletePoint(KDPoint new_point) {}

double KDTree::getNearestDistance(const KDPoint& referance_point) {
    return getNearestDistance(referance_point, rootPtr_, 0);
}

double KDTree::getNearestDistance(const KDPoint& referancePoint,
                                  const std::shared_ptr<KDNode>& node,
                                  const int& depth) {
    if (node == nullptr) {
        return std::numeric_limits<double>::infinity();
    }

    double nearest_dist = std::numeric_limits<double>::infinity();
    bool isLeft = referancePoint[depth] < node->point[depth];

    if (isLeft) {
        nearest_dist = getNearestDistance(
            referancePoint, node->leftChildKDNodePtr, (depth + 1) % dimension_);
    } else {
        nearest_dist =
            getNearestDistance(referancePoint, node->rightChildKDNodePtr,
                               (depth + 1) % dimension_);
    }

    double the_dist = euclidian_squared_dist(referancePoint, node->point);
    nearest_dist = std::min(nearest_dist, the_dist);

    double distWall = std::abs(node->point[depth] - referancePoint[depth]);
    if (nearest_dist >= distWall) {
        if (isLeft) {
            the_dist =
                getNearestDistance(referancePoint, node->rightChildKDNodePtr,
                                   (depth + 1) % dimension_);
        } else {
            the_dist =
                getNearestDistance(referancePoint, node->leftChildKDNodePtr,
                                   (depth + 1) % dimension_);
        }
        nearest_dist = std::min(nearest_dist, the_dist);
    }

    return nearest_dist;
}

double KDTree::euclidian_squared_dist(const KDPoint& point1,
                                      const KDPoint& point2) {
    const double& dx = point1[0] - point2[0];
    const double& dy = point1[1] - point2[1];
    return std::sqrt(dx * dx + dy * dy);
}

KDPoint KDTree::getNearestPoint(const KDPoint& referancePoint) {
    return getNearestPoint(referancePoint, rootPtr_, 0);
}

KDPoint KDTree::getNearestPoint(const KDPoint& referancePoint,
                                const std::shared_ptr<KDNode>& node,
                                const int& depth) {
    if (node == nullptr) {
        return {std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
    }

    KDPoint nearestPoint;
    KDPoint the_point;
    double distMin = std::numeric_limits<double>::max();

    bool isLeft = referancePoint[depth] < node->point[depth];

    if (isLeft) {
        the_point = getNearestPoint(referancePoint, node->leftChildKDNodePtr,
                                    (depth + 1) % dimension_);
    } else {
        the_point = getNearestPoint(referancePoint, node->rightChildKDNodePtr,
                                    (depth + 1) % dimension_);
    }

    double the_dist = euclidian_squared_dist(referancePoint, the_point);
    double node_dist = euclidian_squared_dist(referancePoint, node->point);

    if (node_dist < the_dist) {
        nearestPoint = node->point;
        distMin = node_dist;
    } else {
        nearestPoint = the_point;
        distMin = the_dist;
    }

    double distWall = std::abs(node->point[depth] - referancePoint[depth]);

    if (distMin >= distWall) {
        if (isLeft) {
            the_point =
                getNearestPoint(referancePoint, node->rightChildKDNodePtr,
                                (depth + 1) % dimension_);
        } else {
            the_point =
                getNearestPoint(referancePoint, node->leftChildKDNodePtr,
                                (depth + 1) % dimension_);
        }
        the_dist = euclidian_squared_dist(referancePoint, the_point);
        if (the_dist < distMin) {
            nearestPoint = the_point;
        }
    }

    return nearestPoint;
}

void KDTree::print() { print(rootPtr_, 0); }

void KDTree::print(std::shared_ptr<KDNode> node, const int& depth) {
    if (node == nullptr) {
        return;
    }

    std::cout << "depth : " << depth << " / ";
    print_node(node);
    std::cout << " -> ";
    print_node(node->leftChildKDNodePtr);
    std::cout << " | ";
    print_node(node->rightChildKDNodePtr);
    std::cout << std::endl;

    print(node->leftChildKDNodePtr, (depth + 1) % dimension_);
    print(node->rightChildKDNodePtr, (depth + 1) % dimension_);
}

void KDTree::print_node(const std::shared_ptr<KDNode>& node) {
    if (node == nullptr) {
        std::cout << "NULL";
        return;
    }

    std::cout << "{";
    for (auto p : node->point) {
        std::cout << p << ", ";
    }
    std::cout << "}";
}

}  // end of namespace planning