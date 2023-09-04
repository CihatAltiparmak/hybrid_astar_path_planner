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

#ifndef __NODE_HPP__
#define __NODE_HPP__

namespace planning {

struct Node {
    double x;
    double y;
    double yaw;
    double h_cost;
    double f_cost;
    std::shared_ptr<Node> parent;

    Node() : Node(0.0, 0.0, 0.0) {}
    Node(const double &x, const double &y) : Node(x, y, 0.0) {}
    Node(const double &x, const double &y, const double &yaw)
        : x(x), y(y), yaw(yaw) {}

    Node getNextNode(const double &steering, const double &velocity,
                     const double &wheelbase, const double &dt) const {
        Node next_node;

        double delta_yaw =
            normalize_yaw(velocity * dt * std::tan(steering) / wheelbase);
        next_node.x = x + velocity * std::cos(yaw + delta_yaw) * dt;
        next_node.y = y + velocity * std::sin(yaw + delta_yaw) * dt;
        next_node.yaw = normalize_yaw(yaw + delta_yaw);
        return next_node;
    }

    double normalize_yaw(const double &yaw) const {
        double v = std::fmod(yaw, 2 * M_PI);

        if (v < -M_PI) {
            v += 2.0 * M_PI;
        } else if (v > M_PI) {
            v -= 2.0 * M_PI;
        }

        return v;
    }
};

inline bool operator<(const std::shared_ptr<Node> &n1,
                      const std::shared_ptr<Node> &n2) {
    return n1->f_cost > n2->f_cost;
}

}  // end of namespace planning

#endif