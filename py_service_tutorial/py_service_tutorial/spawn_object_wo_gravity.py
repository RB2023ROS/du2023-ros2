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
        self.sdf_file_path = os.path.join(
            get_package_share_directory('rgbd_world'), 'models',
        )

        self.model_dict = {
            "1": "arm_part",
            "2": "beer",
            "3": "biscuits",
            "4": "book",
            "5": "bowl",
            "6": "create",
            "7": "disk_part",
            "8": "eraser",
            "9": "glue",
            "10": "hammer",
            "11": "plastic_cup",
            "12": "snacks",
            "13": "soap",
            "14": "soap2",
            "15": "soda_can",
            "16": "sticky_notes"
        }

    def send_pause_req(self):

        self.req = Empty.Request()
        self.future = self.physics_client.call_async(self.req)

        return self.future

    def send_spawn_req(self):
        
        # model_name = "beer"
        model_id = input("""Enter Model name Among Below List
        1.arm_part\t2.beer         \t3.biscuits
        4.book     \t5.bowl        \t6.create
        7.disk_part\t8.eraser      \t9.glue
        10.hammer  \t11.plastic_cup\t12.snacks
        13.soap    \t14.soap2      \t15.soda_can
        16.sticky_notes
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

        self.req.initial_pose.orientation.z = 0.707
        self.req.initial_pose.orientation.w = 0.707

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