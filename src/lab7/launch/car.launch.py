import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = '/home/meet/ros2_ws/src/lab7/urdf/car.urdf'
    controller_file = '/home/meet/ros2_ws/src/lab7/config/control.yaml'
    
    # Process the URDF file
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/home/meet/ros2_ws/src/lab7/launch/gazebo.launch.py"),
    )

    # Create Node for robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Create Node for spawning the entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-entity", "car", "-topic", "robot_description"],
        output='screen'
    )

    # RViz
    # Include Gazebo launch file
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/home/meet/ros2_ws/src/lab7/launch/rviz.launch.py"),
    )

    rviz_config_file = '/home/meet/ros2_ws/src/lab7/rviz/car.rviz'
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        gazebo,
        rviz,
        node_robot_state_publisher,
        spawn_entity,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_file],
            output="screen"
        ),
        # rviz_node
    ])