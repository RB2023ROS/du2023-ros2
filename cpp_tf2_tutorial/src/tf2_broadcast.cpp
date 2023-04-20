#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// gazebo_msgs/msg/ModelStates
// 
// # broadcast all model states in world frame
// string[] name                 # model names
// geometry_msgs/Pose[] pose     # desired pose in world frame
// geometry_msgs/Twist[] twist   # desired twist in world frame

using ModelStates = gazebo_msgs::msg::ModelStates;

class OrbitBroadcaster : public rclcpp::Node
{
private:
  rclcpp::Subscription<ModelStates>::SharedPtr m_model_state_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  
public:
  OrbitBroadcaster() : Node("tf2_frame_broadcaster"){
    
    m_model_state_sub = this->create_subscription<ModelStates>(
      "model_states", 10,
      std::bind(&OrbitBroadcaster::model_state_sub_cb, this, std::placeholders::_1)
    );

    // Initialize the transform broadcaster
    m_tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void model_state_sub_cb(const ModelStates::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    std::string model_name = "animated_sphere";
    auto it = find(msg->name.begin(), msg->name.end(), model_name);
    auto index = it - msg->name.begin();

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "animated_sphere";

    t.transform.translation.x = msg->pose[index].position.x;
    t.transform.translation.y = msg->pose[index].position.y;
    t.transform.translation.z = msg->pose[index].position.z;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);

    t.transform.rotation.x = msg->pose[index].orientation.x;
    t.transform.rotation.y = msg->pose[index].orientation.y;
    t.transform.rotation.z = msg->pose[index].orientation.z;
    t.transform.rotation.w = msg->pose[index].orientation.w;

    // Send the transformation
    m_tf_broadcaster->sendTransform(t);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbitBroadcaster>());
  rclcpp::shutdown();
  return 0;
}