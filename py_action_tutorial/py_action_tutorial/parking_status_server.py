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

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from action_tutorials_interfaces.action import Fibonacci
from custom_interfaces.action import Parking
from sensor_msgs.msg import LaserScan

"""Parking.action
#goal definition
bool start_flag
---
#result definition
string message 
---
#feedback definition
float32 distance
"""

class ParkingActionServer(Node):

    def __init__(self):
        super().__init__('parking_action_server')
        
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, queue_size
        )

        self.action_server = ActionServer(
            self, Parking, 'src_parking', self.execute_callback
        )

        self.is_sub = False
        # distance from forward obstacles
        self.f_obs_distance = 100.0

        # distance from L/R obstacles
        self.r_obs_distance = 100.0
        self.l_obs_distance = 100.0

    def sub_callback(self, data):
        
        if is_sub:
            self.f_obs_distance = msg.ranges[60]
            self.r_obs_distance = msg.ranges[30]
            self.l_obs_distance = msg.ranges[90]

    def execute_callback(self, goal_handle):

        is_sub = True

        self.get_logger().info('Executing goal...')

        feedback_msg = Parking.Feedback()

        while self.f_obs_distance > 0.5:
            feedback_msg.distance = self.f_obs_distance
            self.get_logger().info(
                f"Distance from forward obstacle : {self.f_obs_distance}"
            )
            goal_handle.publish_feedback(feedback_msg)

        # for i in range(1, goal_handle.request.order):
        #     feedback_msg.partial_sequence.append(
        #         feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
        #     time.sleep(1)

        goal_handle.succeed()

        result = Parking.Result()
        if abs(self.r_obs_distance - self.l_obs_distance) < 0.5:
            result.message = "Oh... Teach me how you did :0"
        else:
            result.message = "Be careful, Poor Driver! "
        return result


def main(args=None):
    rclpy.init(args=args)

    parking_action_server = ParkingActionServer()

    rclpy.spin(parking_action_server)


if __name__ == '__main__':
    main()