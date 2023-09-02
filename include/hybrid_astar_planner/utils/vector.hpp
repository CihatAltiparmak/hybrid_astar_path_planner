#ifndef __VECTOR_HPP__
#define __VECTOR_HPP__

#include <planner_msgs/msg/point.h>

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