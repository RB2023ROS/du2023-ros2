#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

class LaserSubNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<LaserScan>::SharedPtr laser_subscriber;

  void sub_callback(const LaserScan::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "msg.ranges[0]: '%f'", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "msg.ranges[30]: '%f'", msg->ranges[30]);
    RCLCPP_INFO(this->get_logger(), "msg.ranges[60]: '%f'", msg->ranges[60]);
    RCLCPP_INFO(this->get_logger(), "msg.ranges[90]: '%f'", msg->ranges[90]);
    RCLCPP_INFO(this->get_logger(), "msg.ranges[119]: '%f'\n", msg->ranges[119]);
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

  rclcpp::spin(std::make_shared<LaserSubNode>());

  rclcpp::shutdown();
  return 0;
}