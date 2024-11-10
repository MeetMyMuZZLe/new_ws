cmake_minimum_required(VERSION 3.8)
project(mecanum_drive_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Create the plugin library
add_library(mecanum_drive_plugin SHARED
  src/mecanum_drive_plugin.cpp
)

# Specify dependencies
ament_target_dependencies(mecanum_drive_plugin
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "gazebo_ros"
)

target_link_libraries(mecanum_drive_plugin
  ${GAZEBO_LIBRARIES}
)

# Install
install(TARGETS
  mecanum_drive_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()