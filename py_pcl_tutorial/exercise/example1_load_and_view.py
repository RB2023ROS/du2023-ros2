import pcl
import pcl.pcl_visualization

# Load point cloud file
cloud = pcl.load_XYZRGB('./data/pcl_sub_node.pcd')

# Create pcl built-in viewer
visual = pcl.pcl_visualization.CloudViewing()

##### Visualize APIs #####
# 'ShowColorACloud', 
# 'ShowColorCloud', 
# 'ShowGrayCloud', 
# 'ShowMonochromeCloud', 
# 'WasStopped'

# filename = 'example_save.pcd'
# pcl.save(cloud, filename)

visual.ShowColorCloud(cloud, b'cloud')
v = True
while v:
    v = not(visual.WasStopped())