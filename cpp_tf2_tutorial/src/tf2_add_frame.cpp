#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

const double PI = 3.14159265358979323846;

using namespace std::chrono_literals;

class InverOrbitFrame : public rclcpp::Node
{
private:
  int count = 0;
  double freq_factor = 2*PI/20;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
public:
  InverOrbitFrame() : Node("inverse_orbit_frame_tf2_broadcaster"){
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&InverOrbitFrame::broadcast_timer_callback, this));
  }

  void broadcast_timer_callback(){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "animated_sphere";
    t.child_frame_id = "inverse_orbit_frame";
    
    t.transform.translation.x = cos(-freq_factor * count);
    t.transform.translation.y = sin(-freq_factor * count);
    t.transform.translation.z = 0.5;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    m_tf_broadcaster->sendTransform(t);
    count++;
    count >= 20 ? count = 0 : count;
  }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<InverOrbitFrame>());

  rclcpp::shutdown();
  return 0;
}