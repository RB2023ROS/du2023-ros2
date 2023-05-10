import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TF2Listener(Node):

    def __init__(self):
        super().__init__('tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'sphere_vel', 10)

        # Call timer_cb function every 10 seconds
        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'animated_sphere'
        to_frame_rel = 'world'

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel, to_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Twist()

        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(
            t.transform.translation.y,
            t.transform.translation.x)

        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        self.get_logger().info(f"linear_vel: {msg.linear.x}, angular_vel: {msg.angular.z}")
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TF2Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()