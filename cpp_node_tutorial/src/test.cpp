#include <rclcpp/rclcpp.hpp>

// rclcpp basic node example
int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a node
  auto node = rclcpp::Node::make_shared("example_node_1");

  // Log a message
  RCLCPP_INFO(node->get_logger(), "==== Hello ROS 2 ====");

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}