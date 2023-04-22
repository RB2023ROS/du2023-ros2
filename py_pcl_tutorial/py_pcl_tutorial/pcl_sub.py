import rclpy
from rclpy.node import Node

import pcl
from .pcl_helper import *
from sensor_msgs.msg import PointCloud2

class PCLSubNode(Node):

    def __init__(self):
        super().__init__('pcl_sub_node')

        self.subscription = self.create_subscription(
            PointCloud2, 'pointcloud',
            self.pcl_callback, 10
        )

        self.cb_flag = True

    def pcl_callback(self, pcl_msg):

        # TODO: Convert ROS msg to PCL data
        pcl_data = ros2_to_pcl(pcl_msg)

        if self.cb_flag:
            filename = 'pcl_sub_node.pcd'
            pcl.save(pcl_data, filename)
            self.get_logger().info(f'Save done : {filename}')
            self.cb_flag = False

def main(args=None):
    rclpy.init(args=args)

    node = PCLSubNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
