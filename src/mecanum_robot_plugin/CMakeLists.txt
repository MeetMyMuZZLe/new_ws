# File: CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(mecanum_robot_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Create the plugin library
add_library(combined_mecanum_plugin SHARED
  src/combined_mecanum_plugin.cpp
)

target_include_directories(combined_mecanum_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(combined_mecanum_plugin
  rclcpp
  gazebo_ros
  geometry_msgs
  std_msgs
)

target_link_libraries(combined_mecanum_plugin
  ${GAZEBO_LIBRARIES}
)

# Install the plugin library
install(TARGETS combined_mecanum_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()