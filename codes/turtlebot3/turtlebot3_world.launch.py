import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch config for sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # World file
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world')
    
    # Robot model file
    urdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle', 'model.sdf')

    # Launch Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Create robot spawning commands with proper namespacing
    robot_spawns = []
    robot_positions = [
        ('waffle1', -2.0, -0.5),
        ('waffle2', 2.0, -0.5),
        ('waffle3', -2.0, 0.5),
        ('waffle4', 2.0, 0.5)
    ]

    for robot_name, x, y in robot_positions:
        # Create a group action with namespace
        group = GroupAction([
            # Push namespace for this robot
            PushRosNamespace(robot_name),
            
            # Launch robot state publisher under namespace
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'namespace': robot_name
                }.items()
            ),
            
            # Spawn robot with namespace
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', robot_name,
                     '-file', urdf_file,
                     '-x', str(x),
                     '-y', str(y),
                     '-z', '0.01',
                     '-robot_namespace', robot_name],
                output='screen'
            )
        ])
        robot_spawns.append(group)

    # Create launch description and add actions
    ld = LaunchDescription()

    # Add Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Add all robot spawning groups
    for spawn_action in robot_spawns:
        ld.add_action(spawn_action)

    return ld