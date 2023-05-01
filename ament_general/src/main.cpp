#include "ament_general/my_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}