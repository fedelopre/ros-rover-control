cmake_minimum_required(VERSION 3.8)
project(move_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (if you have any configuration files)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Add executable
add_executable(sistemaoperativo src/sistemaoperativo.cpp)

# Install executable
install(TARGETS sistemaoperativo
  DESTINATION lib/${PROJECT_NAME})

# Add dependencies to the executable
ament_target_dependencies(sistemaoperativo
  rclcpp
  std_msgs
  geometry_msgs
)

# Set C++ standard
target_compile_features(sistemaoperativo PUBLIC cxx_std_20)

# Testing configuration
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