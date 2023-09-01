#ifndef __KD_TREE_HPP__
#define __KD_TREE_HPP__

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>  // std::numeric_limits
#include <memory>
#include <vector>

namespace planning {

using KDPoint = std::vector<double>;

struct KDNode {
    KDPoint point;
    std::shared_ptr<KDNode> rightChildKDNodePtr;
    std::shared_ptr<KDNode> leftChildKDNodePtr;

    KDNode(const KDPoint& point,
           const std::shared_ptr<KDNode>& leftChildKDNodePtr,
           const std::shared_ptr<KDNode>& rightChildKDNodePtr) {
        this->point = point;
        this->rightChildKDNodePtr = rightChildKDNodePtr;
        this->leftChildKDNodePtr = leftChildKDNodePtr;
    }

    KDNode() : KDNode({}, nullptr, nullptr) {}
};

class KDTree {
   private:
    std::shared_ptr<KDNode> rootPtr_;
    int dimension_;
    void insertPoint(const KDPoint&, std::shared_ptr<KDNode>&, const int&);
    double getNearestDistance(const KDPoint&, const std::shared_ptr<KDNode>&,
                              const int&);
    KDPoint getNearestPoint(const KDPoint&, const std::shared_ptr<KDNode>&,
                            const int&);

   public:
    KDTree(std::vector<KDPoint>&, const int&);
    std::shared_ptr<KDNode> build(std::vector<KDPoint>&, const int&, const int&,
                                  const int&) const;
    void insertPoint(KDPoint);
    void deletePoint(KDPoint);
    double getNearestDistance(const KDPoint&);
    KDPoint getNearestPoint(const KDPoint&);

    double euclidian_squared_dist(const KDPoint&, const KDPoint&);

    void print_node(const std::shared_ptr<KDNode>&);
    void print();
    void print(std::shared_ptr<KDNode>, const int&);
};

}  // end of namespace planning

#endif