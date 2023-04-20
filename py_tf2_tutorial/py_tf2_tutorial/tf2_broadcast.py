import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

class OrbitBroadcaster(Node):

    def __init__(self):
        super().__init__('tf2_frame_broadcaster')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.m_model_state_sub = self.create_subscription(
            ModelStates, "model_states", self.handle_turtle_pose, 1
        )

    def handle_turtle_pose(self, msg):

        index = [i for i in range(len(msg.name)) if msg.name[i] == "animated_sphere"][0]

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'animated_sphere'

        t.transform.translation.x = msg.pose[index].position.x
        t.transform.translation.y = msg.pose[index].position.y
        t.transform.translation.z = msg.pose[index].position.z

        t.transform.rotation.x = msg.pose[index].orientation.x
        t.transform.rotation.y = msg.pose[index].orientation.y
        t.transform.rotation.z = msg.pose[index].orientation.z
        t.transform.rotation.w = msg.pose[index].orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OrbitBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()