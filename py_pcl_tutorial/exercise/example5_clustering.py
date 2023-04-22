import pcl
import pcl.pcl_visualization

# Load point cloud file
cloud = pcl.load_XYZRGB('./data/tabletop.pcd')

# Voxel Grid filter
# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

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
axis_min = 0.6
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



##### Visualize APIs #####
# 'ShowColorACloud', 
# 'ShowColorCloud', 
# 'ShowGrayCloud', 
# 'ShowMonochromeCloud', 
# 'WasStopped'
visual.ShowColorCloud(cloud_filtered, b'cloud')
v = True
while v:
    v = not(visual.WasStopped())