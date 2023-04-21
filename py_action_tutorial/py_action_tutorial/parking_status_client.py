# !/usr/bin/env/ python3
#
# Copyright 2023 @RoadBalance
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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.action import Parking

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

class ParkingActionClient(Node):

    def __init__(self):
        super().__init__('parking_action_client')
        self._action_client = ActionClient(self, Parking, 'src_parking')

    def send_goal(self):
        goal_msg = Parking.Goal()
        goal_msg.start_flag = True

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.distance}')

        if feedback.distance > 6.0:
            self.cancel_goal()

    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_callback)

    def goal_canceled_callback(self, future):
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
        else:
            self.get_logger().warning('Goal failed to cancel')


def main(args=None):
    rclpy.init(args=args)

    action_client = ParkingActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()