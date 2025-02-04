cmake_minimum_required(VERSION 3.8)
project(img_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(img_process_node src/image_process.cpp)
target_compile_features(img_process_node PUBLIC c_std_99 cxx_std_17)
#include_directories(include)
ament_target_dependencies(img_process_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

include_directories(${SENSOR_MSGS_INCLUDE_DIRS})
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

ament_export_dependencies(img_process_node 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  # eigen_conversions
  # matplotlibcpp
)


install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})
ament_package()
install(
  DIRECTORY launch config images
  DESTINATION share/${PROJECT_NAME}
)