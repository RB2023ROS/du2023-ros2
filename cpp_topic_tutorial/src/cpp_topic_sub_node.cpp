#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

class LaserSubNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber;

  void sub_callback(const LaserScan::SharedPtr msg){

    

    RCLCPP_INFO(this->get_logger(), "angle_min: %.3f/ angle_max: %.3f/ angle_increment: %.3f/ time_increment: %.3f, scan_time: %.3f",
      msg->angle_min, msg->angle_max, msg->angle_increment, msg->time_increment, msg->scan_time);
  }
public:
  LaserSubNode() : Node("laser_sub_node")
  {
    laser_subscriber = this->create_subscription<LaserScan>(
      "scan", 10,
      std::bind(&LaserSubNode::sub_callback, this, std::placeholders::_1)
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TwistPubNode>());

  rclcpp::shutdown();
  return 0;
}