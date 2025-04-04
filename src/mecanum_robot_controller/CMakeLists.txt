cmake_minimum_required(VERSION 3.8)
project(mecanum_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(mecanum_robot_plugin SHARED
  src/mecanum_robot_plugin.cpp
)

ament_target_dependencies(mecanum_robot_plugin
  "gazebo_ros"
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
)

install(TARGETS
  mecanum_robot_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
