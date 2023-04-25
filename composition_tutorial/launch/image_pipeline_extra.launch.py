#!/usr/bin/env python3
#
# Copyright 2023 RoadBalance Inc.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_node = Node(
        package = 'composition_tutorial',
        executable = 'camera_node',
        name = 'camera_node',
        output='screen',
    )

    watermark_node = Node(
        package = 'composition_tutorial',
        executable = 'watermark_node',
        name = 'watermark_node',
        output='screen',
    )

    image_view_node = Node(
        package = 'composition_tutorial',
        executable = 'image_view_node',
        name = 'image_view_node',
        output='screen',
    )

    return LaunchDescription([
        camera_node,
        watermark_node,
        image_view_node,
    ])