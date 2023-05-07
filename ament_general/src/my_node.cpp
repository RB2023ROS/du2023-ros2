#include "ament_general/my_node.hpp"

MyNode::MyNode() : Node("example_node") {
  timer = this->create_wall_timer(
    200ms, std::bind(&MyNode::timer_callback, this)
  );
}

void MyNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
  count++;
}
