#include <string>
#include <chrono>
#include <vector>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

std::string FILE_PATH = "/home/kimsooyoung/ros2_ws/src/du2023-ros2/cpp_pcl_tutorial/data/pcl_sub_node.pcd";

struct Color{
	float r, g, b;
	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB){}
};

struct BoxQ{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
  float cube_width;
  float cube_height;
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

// Eigen::Vector3f eulerFromRot(const Eigen::Matrix3d &R){

//   float sy = sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
//   bool singular = sy < 1e-6; // If

//   float roll, pitch, yaw;
//   if (!singular){
//       roll = atan2(R(2,1) , R(2,2));
//       pitch = atan2(-R(2,0), sy);
//       yaw = atan2(R(1,0), R(0,0));
//   } else{
//       roll = atan2(-R(1,2), R(1,1));
//       pitch = atan2(-R(2,0), sy);
//       yaw = 0;
//   }

//   return Eigen::Vector3f(roll, pitch, yaw);
// }

void renderPointCloud(pcl::visualization::PCLVisualizer &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color){
  	viewer.addPointCloud<pcl::PointXYZ> (cloud, name);
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderBox(pcl::visualization::PCLVisualizer &viewer, BoxQ box, int id, 
  Color color = Color(1,0,0), float opacity=1){
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    viewer.addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer.addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox(pcl::visualization::PCLVisualizer &viewer, Box box, int id, 
  Color color = Color(1,0,0), float opacity=1){

	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer.addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    //viewer.addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer.addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

/*
  Template function for filtering point cloud
*/
template <typename PointT>
class ProcessPoint {
public:
  ProcessPoint(){};
  ~ProcessPoint(){};

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    SeparateClouds(
      pcl::PointIndices::Ptr inliers, 
        typename pcl::PointCloud<PointT>::Ptr cloud){

    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    
    return segResult;
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ransac_segmentation(
      typename pcl::PointCloud<PointT>::Ptr cloud, 
      const int &maxIterations, const float &distanceThreshold){

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;

    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = 
      SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
  }

  std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    euclidean_clustering(
      typename pcl::PointCloud<PointT>::Ptr cloud, 
      float clusterTolerance, int minSize, int maxSize){

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    int j = 0;
    for (pcl::PointIndices getIndices : clusterIndices){
      typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

      for (int index : getIndices.indices)
          cloudCluster->points.push_back(cloud->points[index]);
      
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
  }

  Box bounding_box(
    typename pcl::PointCloud<PointT>::Ptr cluster){

      // Find bounding box for one of the clusters
      PointT minPoint, maxPoint;
      pcl::getMinMax3D(*cluster, minPoint, maxPoint);

      Box box;
      box.x_min = minPoint.x;
      box.y_min = minPoint.y;
      box.z_min = minPoint.z;
      box.x_max = maxPoint.x;
      box.y_max = maxPoint.y;
      box.z_max = maxPoint.z;

      return box;
  }

  BoxQ pca_bounding_box(
    typename pcl::PointCloud<PointT>::Ptr cluster){
      // Find bounding box for one of the clusters
      // PointT minPoint, maxPoint;
      // pcl::getMinMax3D(*cluster, minPoint, maxPoint);

      // reference : http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
      Eigen::Vector4f pcaCentroid;
      pcl::compute3DCentroid(*cluster, pcaCentroid);
      Eigen::Matrix3f covariance;
      pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
      eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
      std::cout << eigenVectorsPCA << std::endl;
      /// This line is necessary for proper orientation in some cases. The numbers come out the same without it,
      /// But, the signs are different and the box doesn't get correctly oriented in some cases.

      // Transform the original cloud to the origin where the principal components correspond to the axes.
      Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
      projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
      projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
      typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new typename pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

      // Get the minimum and maximum points of the transformed cloud.
      PointT minPoint, maxPoint;
      pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
      const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

      BoxQ box;

      // Eigen::Vector3f eulerVec = eulerFromRot(eigenVectorsPCA);
      const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
      // Eigen::Quaternionf bboxQuaternion = Eigen::AngleAxisf(eulerVec(0)*M_PI, Eigen::Vector3f::UnitX())
      //                                     * Eigen::AngleAxisf(eulerVec(1)*M_PI, Eigen::Vector3f::UnitY())
      //                                     * Eigen::AngleAxisf(eulerVec(2)*M_PI, Eigen::Vector3f::UnitZ());
      
      box.bboxQuaternion = bboxQuaternion;
      //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
      box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

      box.cube_length = maxPoint.x - minPoint.x;
      box.cube_width = maxPoint.y - minPoint.y;
      box.cube_height = maxPoint.z - minPoint.z;

      return box;
  }

};

int main (int argc, char** argv) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (FILE_PATH, *cloud);

  ProcessPoint<pcl::PointXYZ> process_point;

  auto segmentation_pair = process_point.ransac_segmentation(cloud, 100, 0.01);
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZ>(segmentation_pair.first, "obstacles");
  
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> 
  auto cloudClusters = process_point.euclidean_clustering(segmentation_pair.first, 0.07, 300, 25000);
  // viewer.addPointCloud<pcl::PointXYZ>(segmentation_pair.second, "plane");

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (auto cluster : cloudClusters){
      std::cout << "cluster size " << cluster->points.size() << std::endl;
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

      // Normal Box
      // auto box = process_point.bounding_box(cluster);
      // PCA Box 
      auto box = process_point.pca_bounding_box(cluster);

      renderBox(viewer, box, clusterId);
      ++clusterId;
  }


  while (!viewer.wasStopped()) {
      viewer.spinOnce();
  }

  return (0);
}