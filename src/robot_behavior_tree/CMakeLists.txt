cmake_minimum_required(VERSION 3.8)
project(robot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_msgs REQUIRED)
# find_package(robot_serial REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)
set(dependencies
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  robot_msgs
  # robot_serial
  geometry_msgs
)

add_library(aim_enemy_action_bt_node SHARED plugins/action/aim_enemy.cpp)
list(APPEND plugin_libs aim_enemy_action_bt_node)

add_library(go_base_action_bt_node SHARED plugins/action/go_base.cpp)
list(APPEND plugin_libs go_base_action_bt_node)

add_library(update_map_info_action_bt_node SHARED plugins/action/update_map_info.cpp)
list(APPEND plugin_libs update_map_info_action_bt_node)

add_library(if_HP_or_bullet_low_condition_bt_node SHARED plugins/condition/if_HP_or_bullet_low.cpp)
list(APPEND plugin_libs if_HP_or_bullet_low_condition_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

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

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)
ament_export_dependencies(${dependencies})


ament_package()
