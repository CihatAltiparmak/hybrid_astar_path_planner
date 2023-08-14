#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

void basic(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}};
}

void basic2(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}, {7, 4}, {8, 5}};
}

void basic3(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}, {7, 4}, {8, 5},
                 {7, 1}, {8, 2}, {4, 7}, {5, 8}};
}

void basic4(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}, {7, 4}, {8, 5}, {7, 1}, {8, 2},
                 {2, 6}, {3, 7}, {7, 6}, {8, 7}, {7, 8}, {8, 9}};
}

void basic5(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 4}, {8, 5}, {7, 1}, {8, 2}, {2, 6}, {3, 7}, {7, 6}, {8, 7},
                 {7, 8}, {8, 9}, {7, 5}, {8, 6}, {7, 7}, {8, 8}, {9, 9}};
}

void basic6(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}, {7, 4}, {8, 5}, {7, 1}, {8, 2}, {2, 6},
                 {3, 7}, {7, 6}, {8, 7}, {7, 8}, {8, 9}, {7, 5}, {8, 6},
                 {8, 9}, {7, 9}, {6, 9}, {5, 9}, {4, 9}, {3, 9}, {2, 9}};
}

void basic7(std::vector<grid_map::Index>& obstacles) {
    obstacles = {{7, 7}, {8, 8}, {7, 4}, {8, 5}, {7, 1}, {8, 2}, {2, 6},
                 {3, 7}, {7, 6}, {8, 7}, {7, 8}, {8, 9}, {7, 5}, {8, 6},
                 {8, 9}, {7, 9}, {6, 9}, {5, 9}, {4, 9}, {3, 9}, {2, 9},
                 {7, 0}, {8, 1}, {7, 2}, {8, 3}, {7, 3}, {8, 4}};
}

void clearGridMap(grid_map::GridMap& map) {
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);
        map.at("z", *it) = 0.0;
        map.at("obstacle", *it) = 0.0;
    }
}

void addObstaclesToGridMap(grid_map::GridMap& map) {
    std::vector<grid_map::Index> obstacles;
    basic5(obstacles);

    for (auto obIndx : obstacles) {
        map.at("obstacle", obIndx) = 10.0;
        map.at("z", obIndx) = 10.0;
    }
}