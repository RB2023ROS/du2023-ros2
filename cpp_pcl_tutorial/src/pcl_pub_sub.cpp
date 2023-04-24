#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

using Point = pcl::PointXYZI;

using PointCloud2 = sensor_msgs::msg::PointCloud2;

template <typename T>
pcl::PointCloud<T> pcl_processing(const pcl::PointCloud<T> in_pointcloud){
  // Create output pointcloud
  pcl::PointCloud<T> out_pointcloud;

  // Processing
  out_pointcloud = in_pointcloud;

  return out_pointcloud;
}

class PCLPubSubNode : public rclcpp::Node {
private:
  rclcpp::Subscription<PointCloud2>::SharedPtr point_sub;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_pub;

public:
  PCLPubSubNode() : Node("pcl_pub_sub"){
      
      point_sub = this->create_subscription<PointCloud2>(
        "pointcloud", 10, 
        std::bind(&PCLPubSubNode::topic_callback_3d, this, std::placeholders::_1)
      );

      point_pub = this->create_publisher<PointCloud2>("my_point", 10);
  }

  void topic_callback_3d(const PointCloud2::SharedPtr msg) const {    
    // Convert to PCL data type
    pcl::PointCloud<Point> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    // PCL Processing
    pcl::PointCloud<Point> pcl_pointcloud = pcl_processing(point_cloud);
    
    // Convert to ROS data type
    PointCloud2 output;
    pcl::toROSMsg(pcl_pointcloud, output);
    output.header = msg->header;

    // Publish the data
    point_pub -> publish(output);
  }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PCLPubSubNode>());
  
  rclcpp::shutdown();
  return 0;
}