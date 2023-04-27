import rclpy
from rclpy.node import Node

import pcl
from .pcl_helper import *
from sensor_msgs.msg import PointCloud2

class PCLClusterNode(Node):

    def __init__(self):
        super().__init__('pcl_cluster_node')

        self.pc_subscription = self.create_subscription(
            PointCloud2, 'pointcloud',
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

        # Pass Through filter
        passthrough = cloud_voxed.make_passthrough_filter()

        # Assign axis and range to the passthrough filter object.
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = -1.5
        axis_max = 1.1
        passthrough.set_filter_limits(axis_min, axis_max)

        cloud_filtered = passthrough.filter()

        # RANSAC plane segmentation
        # Create the segmentation object
        seg = cloud_filtered.make_segmenter()

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
        extracted_inliers = cloud_filtered.extract(inliers, negative=True)

        # Euclidean Clustering
        # create kd tree for the search method of extraction
        tree = extracted_inliers.make_kdtree()

        # Create a cluster extraction object
        ec = extracted_inliers.make_EuclideanClusterExtraction()

        # Set tolerances for distance threshold 
        # as well as minimum and maximum cluster size (in points)
        ec.set_ClusterTolerance(0.07)
        ec.set_MinClusterSize(10)
        ec.set_MaxClusterSize(25000)

        # Search the k-d tree for clusters
        ec.set_SearchMethod(tree)
        # Extract indices for each of the discovered clusters
        cluster_indices = ec.Extract()

        # Create Cluster-Mask Point Cloud to visualize each cluster separately
        get_color_list.color_list =[]
        cluster_color = get_color_list(len(cluster_indices))

        # Assign a color corresponding to each segmented object in scene
        color_cluster_point_list = []

        for j, indices in enumerate(cluster_indices):
            for i, indice in enumerate(indices):
                color_cluster_point_list.append([extracted_inliers[indice][0],
                                                extracted_inliers[indice][1],
                                                extracted_inliers[indice][2],
                                                rgb_to_float(cluster_color[j])])

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
