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

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/srv/circle_turtle.hpp"

using CircleTurtle = custom_interfaces::srv::CircleTurtle;
using Twist = geometry_msgs::msg::Twist;

/**
 * custom_interfaces/srv/CircleTurtle srv Description.
 * 
 * float32 time   # Turtle will turn during this seconds
 * ---
 * bool success   # Success or Not
 * string message # Any optional message
 */

class TurtleCircleNodeAdvanced : public rclcpp::Node {
private:
  static float moving_time;

  rclcpp::Service<CircleTurtle>::SharedPtr circle_turtle_client;
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher;

  Twist twist_msg = geometry_msgs::msg::Twist();

  void turtle_circle(){
    twist_msg.linear.x = 2.0;
    twist_msg.angular.z = 1.0;

    auto t_start = this->now();
    auto t_now = this->now();
    auto t_interval = (t_now - t_start).seconds();

    while (t_interval < moving_time) {
      t_now = this->now();
      twist_publisher->publish(twist_msg);
      
      if( (int)(t_interval * 1000) % 1000 == 0)
        RCLCPP_INFO(this->get_logger(), "%.2f Seconds Passed", t_interval);
      
      t_interval = (t_now - t_start).seconds();
    }

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    RCLCPP_WARN(this->get_logger(), "Turtle Stop!!");

    twist_publisher->publish(twist_msg);
  }

  void server_callback(const std::shared_ptr<CircleTurtle::Request> request,
                      const std::shared_ptr<CircleTurtle::Response> response){
    moving_time = request->time;
    turtle_circle();

    response->success = true;
    response->message = "Turtle successfully drawed Circle";
  }

public:
  TurtleCircleNodeAdvanced() : Node("turtle_circle_server_advanced"){
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    circle_turtle_client = this->create_service<CircleTurtle>(
      "turtle_circle_advanced",
      std::bind(&TurtleCircleNodeAdvanced::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Turtle Turning Server Started, Waiting for Request...");
  }
};

float TurtleCircleNodeAdvanced::moving_time = 0.0;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurtleCircleNodeAdvanced>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
