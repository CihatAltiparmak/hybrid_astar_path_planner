cmake_minimum_required(VERSION 3.5)
project(hybrid_astar_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(test_plan
  test/test_plan.cpp
  src/hybrid_astar_planner.cpp
  src/path_smoother.cpp
  src/kd_tree.cpp)

ament_target_dependencies(test_plan 
  rclcpp 
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  planner_msgs)

install(TARGETS
  test_plan
  DESTINATION lib/${PROJECT_NAME})

############################

add_executable(test_update_neigbour
  test/test_update_neigbour.cpp
  src/hybrid_astar_planner.cpp
  src/path_smoother.cpp
  src/kd_tree.cpp)

ament_target_dependencies(test_update_neigbour 
  rclcpp 
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  planner_msgs)

install(TARGETS
  test_update_neigbour
  DESTINATION lib/${PROJECT_NAME})

############################

add_executable(test_is_path_valid
  test/test_is_path_valid.cpp
  src/hybrid_astar_planner.cpp
  src/path_smoother.cpp
  src/kd_tree.cpp)

ament_target_dependencies(test_is_path_valid 
  rclcpp 
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  nav_msgs
  planner_msgs)

install(TARGETS
  test_is_path_valid
  DESTINATION lib/${PROJECT_NAME})

#############################

add_executable(test_kd_tree
  test/test_kd_tree.cpp
  src/kd_tree.cpp)

ament_target_dependencies(test_kd_tree 
  rclcpp 
  grid_map_ros
  grid_map_msgs
  visualization_msgs)

install(TARGETS
  test_kd_tree
  DESTINATION lib/${PROJECT_NAME})

############################

add_executable(test_plan_smoothed
  test/test_plan_smoothed.cpp
  src/hybrid_astar_planner.cpp
  src/path_smoother.cpp
  src/kd_tree.cpp)

ament_target_dependencies(test_plan_smoothed 
  rclcpp 
  grid_map_ros
  grid_map_msgs
  visualization_msgs
  planner_msgs
  tf2)

install(TARGETS
  test_plan_smoothed
  DESTINATION lib/${PROJECT_NAME})

# Mark other files for installation
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
