#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]
    
    if controller == "joystick":
        node = Node(
            package="flood_pkg",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    elif controller == "auto":
        node = Node(
            package="flood_pkg",
            executable="auto",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="flood_pkg",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]

def generate_launch_description():
    flood_pkg_path = get_package_share_directory('flood_pkg')

    rviz_path = os.path.join(
        flood_pkg_path, "rviz", "rviz.rviz"
    )
    
    yaml_file_path = os.path.join(
        get_package_share_directory('flood_pkg'),
        'config', 'drone.yaml'
    )

    xacro_file_name = "drone_again.urdf.xacro"

    urdf_file = os.path.join(
        get_package_share_directory("flood_pkg"),
        "urdf", xacro_file_name
    )
    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    load_fan_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fan_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[urdf_file, yaml_file_path],
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_path
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(flood_pkg_path, 'launch', 'dronecontrol.launch.py')
            )
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
        load_fan_controller
    ])
