#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;

class ParkingNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber;
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher;

  void sub_callback(const LaserScan::SharedPtr msg){

    auto distance_forward = msg->ranges[60];

    if(distance_forward > 0.5){
      auto msg = Twist();
      msg.linear.x = 0.5;
      twist_publisher->publish(msg);
    }else{
      auto msg = Twist();
      msg.linear.x = 0.0;
      twist_publisher->publish(msg);
    }
  }
public:
  ParkingNode() : Node("laser_sub_node")
  {
    twist_publisher = this->create_publisher<Twist>("cmd_vel", 10);

    laser_subscriber = this->create_subscription<LaserScan>(
      "scan", 10,
      std::bind(&ParkingNode::sub_callback, this, std::placeholders::_1)
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ParkingNode>());

  rclcpp::shutdown();
  return 0;
}