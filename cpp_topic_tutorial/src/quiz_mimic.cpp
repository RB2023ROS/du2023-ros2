// Copyright 2022 @RoadBalance
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


/*
This is an third example code for ROS 2 rclpy node programming.

Let's learn about those things.

How can place publisher & subscriber in the same Node.
Make turtle2 following turtle1.
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @brief Receive turtle1's pose then create console log.
 * 
 */
using Twist = geometry_msgs::msg::Twist;

class MimicNode : public rclcpp::Node {
private:
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher;
  rclcpp::Subscription<Twist>::SharedPtr twist_subscriber;

  Twist twist_pub_msg;

  /**
   * @brief sub turtle1 then sends them into turtle2 right away.
   * 
   * @param msg turtle1's cmd_vel msg
   */
  void sub_callback(const Twist::SharedPtr msg) {
    // TODO: turtle2를 움직이는 코드를 작성해 보세요. :)
  }

public:
  MimicNode() : Node("mimic_node") {
    // TODO: publisher와 subscriber를 작성하고 subscriber의 callback도 binding합니다.
    // turtle1/cmd_vel의 topic을 받아 turtle2/cmd_vel으로 publish해야 합니다.
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto mimic_node = std::make_shared<MimicNode>();

  rclcpp::spin(mimic_node);
  rclcpp::shutdown();

  return 0;
}
