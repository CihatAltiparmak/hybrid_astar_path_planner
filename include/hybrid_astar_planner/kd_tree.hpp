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

#ifndef __KD_TREE_HPP__
#define __KD_TREE_HPP__

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>  // std::numeric_limits
#include <memory>
#include <vector>

namespace planning
{

using KDPoint = std::vector<double>;

struct KDNode
{
  KDPoint point;
  std::shared_ptr<KDNode> rightChildKDNodePtr;
  std::shared_ptr<KDNode> leftChildKDNodePtr;

  KDNode(
    const KDPoint & point,
    const std::shared_ptr<KDNode> & leftChildKDNodePtr,
    const std::shared_ptr<KDNode> & rightChildKDNodePtr)
  {
    this->point = point;
    this->rightChildKDNodePtr = rightChildKDNodePtr;
    this->leftChildKDNodePtr = leftChildKDNodePtr;
  }

  KDNode()
  : KDNode({}, nullptr, nullptr) {}
};

class KDTree
{
private:
  std::shared_ptr<KDNode> rootPtr_;
  int dimension_;
  void insertPoint(const KDPoint &, std::shared_ptr<KDNode> &, const int &);
  double getNearestDistance(
    const KDPoint &, const std::shared_ptr<KDNode> &,
    const int &);
  KDPoint getNearestPoint(
    const KDPoint &, const std::shared_ptr<KDNode> &,
    const int &);

public:
  KDTree() = default;
  KDTree(std::vector<KDPoint> &, const int &);
  std::shared_ptr<KDNode> build(
    std::vector<KDPoint> &, const int &, const int &,
    const int &) const;
  void insertPoint(KDPoint);
  void deletePoint(KDPoint);
  double getNearestDistance(const KDPoint &);
  KDPoint getNearestPoint(const KDPoint &);

  double euclidian_squared_dist(const KDPoint &, const KDPoint &);

  void print_node(const std::shared_ptr<KDNode> &);
  void print();
  void print(std::shared_ptr<KDNode>, const int &);
};

}  // end of namespace planning

#endif
