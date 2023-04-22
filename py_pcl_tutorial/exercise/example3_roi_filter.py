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