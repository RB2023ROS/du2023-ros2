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

#include <cmath>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "custom_interfaces/srv/turtle_jail.hpp"

using namespace std::chrono_literals;

using Pose = turtlesim::msg::Pose;
using TurtleJail = custom_interfaces::srv::TurtleJail;
using TeleportAbsolute = turtlesim::srv::TeleportAbsolute;

/**
 * custom_interfaces/srv/TurtleJail srv Description.
 * 
 * float32 width
 * float32 height
 * ---
 * bool success
 */

class TurtleJailNode : public rclcpp::Node {
private:
  float cur_theta = 0.0;

  float jail_width = 6.0;
  float jail_height = 6.0;

  rclcpp::Client<TeleportAbsolute>::SharedPtr teleport_client;
  rclcpp::Service<TurtleJail>::SharedPtr jail_server;
  rclcpp::Subscription<Pose>::SharedPtr pose_subscriber;

  TeleportAbsolute::Request::SharedPtr teleport_request = std::make_shared<TeleportAbsolute::Request>();

  void server_callback(const std::shared_ptr<TurtleJail::Request> request,
                      const std::shared_ptr<TurtleJail::Response> response){
    jail_width = request->width;
    jail_height = request->height;

    RCLCPP_INFO(this->get_logger(), "Jail Size Update to %f / %f", jail_width, jail_height);

    response->success = true;
  }

  auto send_request(){
    teleport_request->x = 6.0;
    teleport_request->y = 6.0;
    teleport_request->theta = cur_theta;

    return teleport_client->async_send_request(teleport_request);
  }

  void sub_callback(const Pose::SharedPtr msg) {
    if(abs(msg->x - 6.0) > jail_width || abs(msg->y - 6.0) > jail_height){
      cur_theta = msg->theta;
      RCLCPP_INFO(this->get_logger(), "You can't go out Turtle! :(");
      send_request();
    }
  }

public:
  TurtleJailNode() : Node("turtle_circle_server_advanced"){
    // service client init
    teleport_client = this->create_client<TeleportAbsolute>("turtle1/teleport_absolute");

    while (!teleport_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),"Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // service server init
    jail_server = this->create_service<TurtleJail>(
      "turtle_jail_size",
      std::bind(&TurtleJailNode::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // topic subscriber init
    pose_subscriber = this->create_subscription<Pose>(
      "turtle1/pose", 10,
      std::bind(&TurtleJailNode::sub_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Turtle Jail Server Started, Waiting for Request...");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurtleJailNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
