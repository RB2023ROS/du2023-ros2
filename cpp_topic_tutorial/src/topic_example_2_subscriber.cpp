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
This is second example code for ROS 2 topic subscriber.

Let's learn about those things.

Create topic subscriber then check the value from that with ros2 command line tools.
Listen to pose of turtle in the turtlesim.
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using Pose = turtlesim::msg::Pose;

/**
 * @brief Receive turtle1's pose then create console log.
 * 
 */
class TurtlePoseSubNode : public rclcpp::Node {
private:
  rclcpp::Subscription<Pose>::SharedPtr pose_subscriber;

  void sub_callback(const Pose::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "x: %.3f/ y: %.3f/ theta: %.3f/ lin_vel: %.3f, ang_vel: %.3f",
      msg->x, msg->y, msg->theta, msg->linear_velocity, msg->angular_velocity);
  }

public:
  TurtlePoseSubNode() : Node("turtlepose_sub_node") {
    pose_subscriber = this->create_subscription<Pose>(
      "turtle1/pose", 10,
      std::bind(&TurtlePoseSubNode::sub_callback, this, std::placeholders::_1)
    );
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlePoseSubNode>());
  rclcpp::shutdown();

  return 0;
}