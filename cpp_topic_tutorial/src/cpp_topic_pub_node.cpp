#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TwistPubNode : public rclcpp::Node
{
private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;

  void cmd_vel_pub(){
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = 0.5;
    msg.angular.z = 0.5;

    twist_publisher->publish(msg);
  }
public:
  TwistPubNode() : Node("twist_pub_node")
  {
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    publish_timer = this->create_wall_timer(
      500ms, 
      std::bind(&TwistPubNode::cmd_vel_pub, this)
    );

    // auto publish_timer = this->create_wall_timer(
    //   500ms,
    //   [twist_publisher]() {
    //     auto msg = geometry_msgs::msg::Twist();
    //     msg.linear.x = 0.5;
    //     msg.angular.z = 0.5;
    //     twist_publisher->publish(msg);
    //   }
    // )
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TwistPubNode>());

  rclcpp::shutdown();
  return 0;
}