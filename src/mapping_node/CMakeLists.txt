cmake_minimum_required(VERSION 3.8)
project(mapping_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare the executable
add_executable(mapping_node src/map.cpp)
target_compile_features(mapping_node PUBLIC cxx_std_17)

# Link dependencies to the executable
ament_target_dependencies(mapping_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install target
install(TARGETS mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Required for ament
ament_package()
