# !/usr/bin/env/ python3
#
# Copyright 2022 @RoadBalance
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

from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class SpawnClientNode(Node):

    def __init__(self):
        super().__init__('gazebo_model_spawner')

        self.physics_client = self.create_client(Empty, 'pause_physics')
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')

        while not self.physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('pause_physics service not available, waiting again...')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('spawn_entity service not available, waiting again...')

        # Get urdf path
        self.sdf_file_path = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')

        self.model_dict = {
            "1": "chair_1",
            "2": "chair_2",
            "3": "chair_3",
            "4": "labtop_mac_1",
            "5": "labtop_mac_2",
            "6": "labtop_mac_3",
            "7": "cup_green",
            "8": "cup_blue",
            "9": "cup_yellow",
            "10": "cup_paper",
            "11": "lamp_table_large",
            "12": "lamp_table_small",
            "13": "side_table_1",
            "14": "side_table_3",
            "15": "side_table_set_1",
            "16": "side_table_set_2",
            "17": "table_dining",
            "18": "cafe_table",
        }

    def send_pause_req(self):

        self.req = Empty.Request()
        self.future = self.physics_client.call_async(self.req)

        return self.future

    def send_spawn_req(self):
        
        # model_name = "beer"
        model_id = input("""Enter Model name Among Below List
        1.chair_1            \t2.chair_2           \t3.chair_3
        4.labtop_mac_1       \t5.labtop_mac_2      \t6.labtop_mac_3
        7.cup_green          \t8.cup_blue          \t9.cup_yellow
        10.cup_paper         \t11.lamp_table_large \t12.lamp_table_small
        13.side_table_1      \t14.side_table_3     \t15.side_table_set_1
        16.side_table_set_2  \t17.table_dining     \t18.cafe_table
        [Type your choice]: """)

        model_name = self.model_dict[model_id]
        model_path = os.path.join(self.sdf_file_path, model_name)

        with open (model_path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')

        self.req = SpawnEntity.Request()
        self.req.name = model_name
        self.req.xml = model_xml

        self.req.initial_pose.position.x = 1.0
        self.req.initial_pose.position.y = 0.0
        self.req.initial_pose.position.z = 1.0

        # self.req.initial_pose.orientation.z = 0.707
        # self.req.initial_pose.orientation.w = 0.707

        self.get_logger().debug('==== Sending service request to `/spawn_entity` ====')
        self.future = self.spawn_client.call_async(self.req)

        return self.future

def future_pending_logic(name, node, future):

    if future.done():
        try:
            response = future.result()
        except Exception:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception()
            )
        else:
            node.get_logger().info(f'==== {name} Service Ongoing ====')
        finally:
            node.get_logger().warn(f'==== {name} Service Finished ====')

def main(args=None):

    rclpy.init(args=args)
    robot_spawn_node = SpawnClientNode()

    # physics pause request
    pause_future = robot_spawn_node.send_pause_req()
    rclpy.spin_until_future_complete(robot_spawn_node, pause_future)
    future_pending_logic("Pause",robot_spawn_node, pause_future)

    # spawn model request
    spawn_future = robot_spawn_node.send_spawn_req()
    rclpy.spin_until_future_complete(robot_spawn_node, spawn_future)
    future_pending_logic("Spawn",robot_spawn_node, spawn_future)

    robot_spawn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()