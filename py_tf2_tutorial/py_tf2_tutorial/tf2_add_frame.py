
import rclpy
import numpy as np
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class InverOrbitFrame(Node):

   def __init__(self):
        super().__init__('inverse_orbit_frame_tf2_broadcaster')
        
        self.count = 0
        self.freq_factor = 2* np.pi / 20;

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'animated_sphere'
       t.child_frame_id = 'inverse_orbit_frame'

       t.transform.translation.x = np.cos(-self.count * self.freq_factor)
       t.transform.translation.y = np.sin(-self.count * self.freq_factor)
       t.transform.translation.z = 0.5

       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)
       self.count += 1
       if self.count >= 20:
           self.count = 0

def main():
    rclpy.init()
    node = InverOrbitFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()