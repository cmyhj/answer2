cmake_minimum_required(VERSION 3.8)
project(robot_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

include_directories(include include/msg_serializer include/thirdparty/serialib include/include/serialPro)
add_subdirectory(include/thirdparty/serialib)
add_executable(robot_serial src/robot_serial_node.cpp)
target_link_libraries(robot_serial serialib)
ament_target_dependencies(robot_serial
        rclcpp
        geometry_msgs
        robot_msgs
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

install(TARGETS robot_serial
        DESTINATION lib/${PROJECT_NAME})

ament_package()
