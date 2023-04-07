import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_ex_node = Node(
        package='py_param_tutorial',
        executable='param_example',
        name='param_example',
        output='screen',
        parameters=[
            {'string_param': 'Hello'},
            {'int_param': 112},
        ],
    )

    return LaunchDescription([
        param_ex_node
    ])