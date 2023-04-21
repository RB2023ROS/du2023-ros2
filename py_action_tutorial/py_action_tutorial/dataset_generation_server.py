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

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity

from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from custom_interfaces.action import DataGen
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image 

angle_list = [
    [ 0.7070727, 0.0, 0.0, 0.7071408 ],
    [ 0.7070727, 0.0, 0.0, -0.7071408 ],
    [ 0.0, 0.8509035, 0.0, 0.525322 ],
    [ 0.0, -0.8509035, 0.0, 0.525322 ],
    [ 0.0, 0.0, 0.8509035, 0.525322 ],
    [ 0.0, 0.0, -0.8509035, 0.525322 ],
]

class DatasetGen(Node):

    def __init__(self):
        super().__init__('parking_action_server')

        self.img_topic_name = "/rgb_cam/rgb_cam/image_raw"
        # Get urdf path
        self.sdf_file_path = os.path.join(
            get_package_share_directory('rgbd_world'), 'models',
        )

        ####### one topic sub #######
        self.br = CvBridge()
        self.img_subscriber = self.create_subscription(
            Image, self.img_topic_name, self.img_sub_callback, 10
        )
        
        ####### three service clients #######
        self.physics_client = self.create_client(Empty, 'pause_physics')
        while not self.physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('pause_physics service not available, waiting again...')

        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('spawn_entity service not available, waiting again...')

        self.delete_client = self.create_client(DeleteEntity, 'delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('delete_entity service not available, waiting again...')

        ####### one action server #######
        self.action_server = ActionServer(
            self, DataGen, 'dataset_generation', 
            self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.rate = self.create_rate(0.3)

        self.is_request = False
        self.model_name = None
        self.x, self.y, self.z = None, None, None

        _ = self.physics_client.call_async(Empty.Request())
        self.get_logger().info('pause_physics done')

    def send_delete_req(self):

        model_name = "beer"

        req = DeleteEntity.Request()
        req.name = model_name

        return self.delete_client.call(req)

    def send_spawn_req(self, i):

        model_name = self.model_name

        model_path = os.path.join(self.sdf_file_path, model_name)

        with open (model_path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml

        req.initial_pose.position.x = self.x
        req.initial_pose.position.y = self.y
        req.initial_pose.position.z = self.z

        angle = angle_list[i]

        req.initial_pose.orientation.x = angle[0]
        req.initial_pose.orientation.y = angle[1]
        req.initial_pose.orientation.z = angle[2]
        req.initial_pose.orientation.w = angle[3]

        return self.spawn_client.call(req)

    def img_sub_callback(self, data):

        if self.is_request:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
            self.rate.sleep()

            file_name = str(self.get_clock().now().to_msg().sec) + '.png'
            cv2.imwrite(file_name, current_frame)
            self.get_logger().info(f'Image saved in {file_name}')
            
            self.is_request = False

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        feedback_msg = DataGen.Feedback()

        for i in range(len(angle_list)):
            _ = self.send_spawn_req(i)
            self.get_logger().info('spawn_entity done')
            self.rate.sleep()
            
            self.is_request = True
            self.rate.sleep()

            _ = self.send_delete_req()
            self.get_logger().info('delete_entity done')
            self.rate.sleep()

            feedback_msg.message = "Done Ongoing"
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = DataGen.Result()
        result.message = "Good"

        return result
    
    def goal_callback(self, goal_request):
        # TODO: print goal_request  
        self.model_name = goal_request.model_name
        self.x, self.y, self.z = goal_request.x, goal_request.y, goal_request.z

        self.get_logger().info(f"""
        Received goal request
        model_name: {self.model_name}
            x: {self.x}
            y: {self.y}
            z: {self.z}
        """)

        return rclpy.action.GoalResponse.ACCEPT

def main(args=None):

    rclpy.init(args=args)

    # MultiThreadedExecutor  ref
    # https://url.kr/x4kf2b
    parking_action_server = DatasetGen()
    executor = MultiThreadedExecutor()
    executor.add_node(parking_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        parking_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        executor.shutdown()
        parking_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()