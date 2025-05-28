cmake_minimum_required(VERSION 3.5)
project(groot_test_cpp)

# Default to C++14 for ROS2 Foxy
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Executable
add_executable(groot_test_node src/groot_test_node.cpp)
ament_target_dependencies(groot_test_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
)
add_executable(groot_test_nav2_node src/groot_test_nav2_node.cpp)
ament_target_dependencies(groot_test_nav2_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
)

add_executable(nav2_zmq_bridge src/nav2_zmq_bridge.cpp)
ament_target_dependencies(nav2_zmq_bridge
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  geometry_msgs
  nav2_msgs
)

# Install executables
install(TARGETS
  groot_test_node
  groot_test_nav2_node
  nav2_zmq_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
