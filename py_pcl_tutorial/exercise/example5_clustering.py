import pcl
from pcl_helper_ex import *
import pcl.pcl_visualization

import numpy as np

# Load point cloud file
cloud = pcl.load_XYZRGB('./data/pcl_sub_node.pcd')

# eliminate color from cloud for clustering
while_cloud = XYZRGB_to_XYZ(cloud)

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

# Create pcl built-in viewer
visual = pcl.pcl_visualization.CloudViewing()

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

print(dir(extracted_inliers))

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

##### Visualize APIs #####
# 'ShowColorACloud', 
# 'ShowColorCloud', 
# 'ShowGrayCloud', 
# 'ShowMonochromeCloud', 
# 'WasStopped'
visual.ShowColorCloud(cluster_cloud, b'cloud')
v = True
while v:
    v = not(visual.WasStopped())