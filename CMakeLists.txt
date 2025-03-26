cmake_minimum_required(VERSION 3.16)
project(dsor_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# -------------------- (Newly added) --------------------
# Example: If dsor.hpp is in the include directory:
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Build example_node.cpp to create an executable named example_node
add_executable(example_node src/example_node.cpp)
# Link the required dependencies for build/run using ament_target_dependencies()
ament_target_dependencies(example_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

# Link the PCL libraries as well (e.g., pcl_filters, pcl_io, etc.)
target_link_libraries(example_node
  ${PCL_LIBRARIES}
)

# Specify the installation path → allows running via "ros2 run dsor_filter example_node"
install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_pcd src/test_pcd.cpp)
ament_target_dependencies(test_pcd rclcpp sensor_msgs pcl_conversions pcl_ros)
target_link_libraries(test_pcd ${PCL_LIBRARIES})
install(TARGETS test_pcd DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------

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
