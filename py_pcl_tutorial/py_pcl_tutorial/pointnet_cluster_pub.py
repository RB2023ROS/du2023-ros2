import rclpy
from rclpy.node import Node

import pcl
from .pcl_helper import *
from sensor_msgs.msg import PointCloud2

class PCLClusterNode(Node):

    def __init__(self):
        super().__init__('pcl_cluster_node')

        self.pc_subscription = self.create_subscription(
            PointCloud2, 'sensor_stick/depth_camera/points',
            self.pcl_callback, 10
        )

        self.cluster_publisher = self.create_publisher(
            PointCloud2, 'cluster', 10
        )

        self.get_logger().info('PCL Cluster Node has been started')

        self.cb_flag = True

    def pcl_callback(self, pcl_msg):

        # TODO: Convert ROS msg to PCL data
        pcl_data = ros2_to_pcl(pcl_msg)

        # eliminate color from cloud for clustering
        while_cloud = XYZRGB_to_XYZ(pcl_data)

        # Voxel Grid filter
        # Create a VoxelGrid filter object for our input point cloud
        vox = while_cloud.make_voxel_grid_filter()

        # Choose a voxel (also known as leaf) size
        # Note: this (1) is a poor choice of leaf size   
        # Experiment and find the appropriate size!
        LEAF_SIZE = 0.01

        # Set the voxel (or leaf) size  
        vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

        # Call the filter function to obtain the resultant downsampled point cloud
        cloud_voxed = vox.filter()

        # RANSAC plane segmentation
        # Create the segmentation object
        seg = cloud_voxed.make_segmenter()

        # Set the model you wish to fit
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)

        # Max distance for a point to be considered fitting the model
        # Experiment with different values for max_distance 
        # for segmenting the table
        max_distance = 0.01
        seg.set_distance_threshold(max_distance)

        # Call the segment function to obtain set of inlier indices and model coefficients
        inliers, coefficients = seg.segment()

        # Extract inliers
        extracted_inliers = cloud_voxed.extract(inliers, negative=True)

        # inliers into list for ROS 2 Conversion
        color_cluster_point_list = []
        # cloud size color generator
        color = size_color_gen(extracted_inliers)

        for i, indices in enumerate(extracted_inliers):
            color_cluster_point_list.append([extracted_inliers[i][0],
                                            extracted_inliers[i][1],
                                            extracted_inliers[i][2],
                                            rgb_to_float(color)])
            
        #Create new cloud containing all clusters, each with unique color
        cluster_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_cloud.from_list(color_cluster_point_list)

        # Convert PCL data to ROS 2 messages
        cluster_cloud_msg = pcl_to_ros2(cluster_cloud, now=self.get_clock().now())
        self.cluster_publisher.publish(cluster_cloud_msg)

def main(args=None):
    rclpy.init(args=args)

    node = PCLClusterNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
