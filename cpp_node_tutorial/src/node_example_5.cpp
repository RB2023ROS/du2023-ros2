// Copyright 2022 @RoadBalance
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

/**
 * @brief Class Type RCLCPP Node
 * 
 */
class NodeClass: public rclcpp::Node {
private:
  size_t count;
  rclcpp::TimerBase::SharedPtr timer;

  /**
   * @brief below method will invoked repetitively by timer
   * 
   */
  void timer_callback() {
    RCLCPP_DEBUG(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
    RCLCPP_INFO(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
    RCLCPP_WARN(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
    RCLCPP_ERROR(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
    RCLCPP_FATAL(this->get_logger(), "==== Hello ROS 2 : %d ====", count);
    count++;
  }

public:
  NodeClass() : Node("example_node_5") {
    timer = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&NodeClass::timer_callback, this)
    );
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NodeClass>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
