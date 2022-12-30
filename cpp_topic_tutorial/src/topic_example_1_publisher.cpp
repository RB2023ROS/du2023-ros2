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
This is first example code for ROS 2 topic publisher.

Let's learn about those things.

Create topic publisher then check the value from that with ros2 command line tools.
Try control turtle in the turtlesim through turtlesim node.
*/

#include <memory>
#include <random>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

std::random_device rd;
std::mt19937 mersenne(rd());
std::uniform_real_distribution<> ang_dis(-1.5707, 1.5707);
std::uniform_real_distribution<> lin_dis(0.0, 1.0);

/**
 * @brief Publish random velocity for turtle1
 * 
 */
class TwistPubNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::TimerBase::SharedPtr timer;

  // declaring non-static data members as 'auto'
  // https://stackoverflow.com/questions/11302981/c11-declaring-non-static-data-members-as-auto
  geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

  void timer_callback() {
    auto linear_vel = lin_dis(mersenne);
    auto angular_vel = ang_dis(mersenne);

    RCLCPP_INFO(get_logger(), "ang_dis : %f, lin_dis: %f", angular_vel, linear_vel);

    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;

    twist_publisher->publish(msg);
  }

public:
  TwistPubNode() : Node("twist_pub_node") {
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TwistPubNode::timer_callback, this)
    );
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPubNode>());
  rclcpp::shutdown();

  return 0;
}
