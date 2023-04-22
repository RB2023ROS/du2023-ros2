import pcl
import pcl.pcl_visualization

# Load point cloud file
cloud = pcl.load_XYZRGB('./data/tabletop.pcd')

# Create pcl built-in viewer
visual = pcl.pcl_visualization.CloudViewing()

##### Visualize APIs #####
# 'ShowColorACloud', 
# 'ShowColorCloud', 
# 'ShowGrayCloud', 
# 'ShowMonochromeCloud', 
# 'WasStopped'

visual.ShowColorCloud(cloud, b'cloud')
v = True
while v:
    v = not(visual.WasStopped())