#include <hybrid_astar_planner/kd_tree.hpp>

int main() {
    std::vector<planning::KDPoint> pl = {
        {2, 2}, {1, 2}, {2, 1}, {3, 0}, {3, 3}};
    planning::KDTree kd_tree(pl, 2);

    kd_tree.print();

    std::cout << "######################" << std::endl;
    kd_tree.insertPoint({2, 13});
    kd_tree.print();

    planning::KDPoint np = kd_tree.getNearestPoint({2, 1});
    std::cout << "nearest dist : " << kd_tree.getNearestDistance({-14, -14})
              << std::endl;
    std::cout << "nearest point : "
              << "(" << np[0] << ", " << np[1] << ")" << std::endl;
}