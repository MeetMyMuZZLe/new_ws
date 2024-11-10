import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('project')

    # URDF file paths
    arena_urdf = os.path.join(pkg_dir, 'urdf', 'ARENA.urdf')
    ball_urdf = os.path.join(pkg_dir, 'urdf', 'Ball.urdf')

    # Process the arena URDF file with xacro
    arena_doc = xacro.parse(open(arena_urdf))
    xacro.process_doc(arena_doc)
    arena_description = arena_doc.toxml()

    # Process the ball URDF file with xacro
    ball_doc = xacro.parse(open(ball_urdf))
    xacro.process_doc(ball_doc)
    ball_description = ball_doc.toxml()

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
            output="screen",
        ),
        
        # Spawn the arena
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "arena",
                "-b",
                "-topic", "/robot_description"
            ],
            output="screen",
        ),
        
        # Publish the arena state
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{'robot_description': arena_description}]
        ),
        
        # Spawn the ball at (0, 0, 0.5)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "ball",
                "-file", ball_urdf,
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.5"
            ],
            output="screen",
        )
    ])