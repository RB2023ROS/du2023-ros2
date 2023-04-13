# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler

from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="start RViz automatically with the launch file",
    )

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_path = os.path.join(get_package_share_directory('src_gazebo'))
    world_path = os.path.join(pkg_path, 'worlds', 'garage.world')
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("src_gazebo"), "urdf", "src_body.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.06',
                    '-Y', '0.0',
                    '-entity', 'src_body'
                ],
        output='screen')

    # ROS 2 controller
    load_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
        output="screen",
    )

    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller"],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Racecar controller launch
    src_gazebo_controller = Node(
        package='src_controller',
        executable='src_gazebo_controller',
        output='screen',
        parameters=[],
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'gazebo.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz"))
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        arg_show_rviz,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_forward_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_forward_position_controller,
                on_exit=[load_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[src_gazebo_controller],
            )
        ),

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        spawn_entity,
        rqt_robot_steering,
        # rviz_node,
    ])