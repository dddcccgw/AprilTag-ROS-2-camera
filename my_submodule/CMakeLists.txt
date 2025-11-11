cmake_minimum_required(VERSION 3.5)

project(my_submodule)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(my_submodule_lib SHARED
  src/my_submodule.cpp
)

target_include_directories(my_submodule_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_submodule_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  apriltag_msgs
  Eigen3
)

# Register as ROS component
rclcpp_components_register_node(my_submodule_lib PLUGIN "MySubmodule" EXECUTABLE "my_submodule_node")

# Install targets
install(TARGETS my_submodule_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
