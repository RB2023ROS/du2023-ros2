#include <string>
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

std::string FILE_PATH = "/home/kimsooyoung/ros2_ws/src/du2023-ros2/cpp_pcl_tutorial/data/pcl_sub_node.pcd";

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr  voxel_downsample(
  typename pcl::PointCloud<PointT>::Ptr cloud, const float &filterRes){

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  // get end time and console output
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudFiltered;
}

int main (int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (FILE_PATH, *cloud);

  auto voxel_cloud = voxel_downsample<pcl::PointXYZ>(cloud, 0.1);
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZ>(voxel_cloud, "cloud");
  while (!viewer.wasStopped()) {
      viewer.spinOnce();
  }
  
  return (0);
}