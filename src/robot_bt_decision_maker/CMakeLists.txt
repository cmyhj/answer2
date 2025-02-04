cmake_minimum_required(VERSION 3.8)
project(robot_bt_decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_behavior_tree REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
add_executable(robot_bt_decision_maker_node src/robot_bt_decision_maker.cpp)
ament_target_dependencies(robot_bt_decision_maker_node rclcpp behaviortree_cpp_v3 robot_behavior_tree nav2_behavior_tree tf2_ros)

install(TARGETS
        robot_bt_decision_maker_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY behavior_trees launch
        DESTINATION share/${PROJECT_NAME})
