cmake_minimum_required(VERSION 3.8)
project(system_health_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(system_monitor_node src/system_monitor_node.cpp)

# Link libraries to the executable
ament_target_dependencies(
  system_monitor_node
  "rclcpp"
  "std_msgs"
)

# Install the executable
install(
  TARGETS system_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()