#include <string>
#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

std::string FILE_PATH = "/home/kimsooyoung/ros2_ws/src/du2023-ros2/cpp_pcl_tutorial/data/pcl_sub_node.pcd";

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr roi_filter(
  typename pcl::PointCloud<PointT>::Ptr cloud, const float &min_z, const float &max_z){

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

  // Create Object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);                
  pass.setFilterFieldName("z");             // filter axis field name
  pass.setFilterLimits(min_z, max_z);       // min, max axis range
  //pass.setFilterLimitsNegative (true);     // true or false
  pass.filter(*cloudFiltered);              // extract

  // get end time and console output
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudFiltered;
}

int main (int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (FILE_PATH, *cloud);

  auto voxel_cloud = roi_filter<pcl::PointXYZ>(cloud, -1.1, 1.1);
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZ>(voxel_cloud, "cloud");
  while (!viewer.wasStopped()) {
      viewer.spinOnce();
  }
  
  return (0);
}