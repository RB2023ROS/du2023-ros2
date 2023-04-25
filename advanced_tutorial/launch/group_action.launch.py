#!/usr/bin/env python3
#
# Copyright 2022 RoadBalance Inc.
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

from launch import LaunchDescription
from launch.actions import LogInfo, GroupAction, DeclareLaunchArgument

from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='whether to use namespace or not'
    )


    turtle_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace
        ),
        Node(
            package = 'turtlesim',
            executable = 'turtlesim_node',
            name = 'turtlesim_node',
            output='screen',
        ),
        Node(
            package = 'turtlesim',
            executable = 'turtle_teleop_key',
            name = 'turtle_teleop_key',
            output='screen',
        ),
    ])


    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_namespace_cmd,

            turtle_group,
        ]
    )