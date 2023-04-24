#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>


std::string FILE_PATH = "/home/kimsooyoung/ros2_ws/src/du2023-ros2/cpp_pcl_tutorial/data/pcl_sub_node.pcd";

int main (int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (FILE_PATH, *cloud);
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  while (!viewer.wasStopped()) {
      viewer.spinOnce();
  }

  // pcl::visualization::CloudViewer viewer("Cloud Viewer");
  // viewer.showCloud(cloud, "cloud");
  // int cnt = 0;
  // while (!viewer.wasStopped()) {
  //     cnt++;
  // }

  return (0);
}