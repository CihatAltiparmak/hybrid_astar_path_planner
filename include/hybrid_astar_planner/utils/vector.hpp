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

#ifndef __VECTOR_HPP__
#define __VECTOR_HPP__

#include <planner_msgs/msg/point.hpp>

namespace planning {

struct Vector2d {
    std::vector<double> data;
    Vector2d(const double& x, const double& y) : data({x, y}) {}
    Vector2d(const planner_msgs::msg::Point& point)
        : data({point.x, point.y}) {}
    Vector2d(const std::vector<double>& data) : data({data[0], data[1]}) {}
    Vector2d() : Vector2d(0.0, 0.0) {}

    double& operator[](const int& index) { return data[index]; }

    double operator[](const int& index) const { return data[index]; }

    Vector2d& operator-=(const Vector2d& other_vector) {
        this->data[0] -= other_vector.data[0];
        this->data[1] -= other_vector.data[1];

        return *this;
    }

    Vector2d operator-(const Vector2d& other_vector) const {
        Vector2d result_vector = *this;
        result_vector -= other_vector;
        return result_vector;
    }

    Vector2d& operator+=(const Vector2d& other_vector) {
        this->data[0] += other_vector.data[0];
        this->data[1] += other_vector.data[1];

        return *this;
    }

    Vector2d operator+(const Vector2d& other_vector) const {
        Vector2d result_vector = *this;
        result_vector += other_vector;
        return result_vector;
    }

    double norm2() {
        const double& d1 = data[0];
        const double& d2 = data[1];
        return std::sqrt(d1 * d1 + d2 * d2);
    }
};

inline Vector2d operator*(const double& a, const Vector2d& vector2d) {
    Vector2d result_vector;
    result_vector.data[0] = a * vector2d.data[0];
    result_vector.data[1] = a * vector2d.data[1];

    return result_vector;
}

inline Vector2d operator/(const Vector2d& vector2d, const double& a) {
    Vector2d result_vector;
    result_vector.data[0] = vector2d.data[0] / a;
    result_vector.data[1] = vector2d.data[1] / a;

    return result_vector;
}

}  // end of namespace planning

#endif