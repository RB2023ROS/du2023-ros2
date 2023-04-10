import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('py_param_tutorial'), 'config', 'params.yaml'
    )
    
    param_ex_node = Node(
        package = 'py_param_tutorial',
        # namespace 지정!
        namespace = 'robot1',
        executable = 'param_example',
        name = 'param_example',
        output='screen',
        parameters = [config]
    )

    return LaunchDescription([
        param_ex_node
    ])