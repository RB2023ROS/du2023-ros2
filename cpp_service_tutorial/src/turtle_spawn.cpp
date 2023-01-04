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
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;
using Spawn = turtlesim::srv::Spawn;

template <typename T>
void get_user_input(const std::string &str, T &src){
  while(1){
    std::cout << str;
    std::cin >> src;

    if (std::cin.fail()){
      std::cin.clear();
      std::cin.ignore(32767, '\n');
      std::cout << "Invalid input detected" << std::endl;
    } else {
      std::cin.ignore(32767, '\n');
      return;
    }
  }
}

class SpawnTurtle : public rclcpp::Node {
private:
  rclcpp::Client<Spawn>::SharedPtr spawn_client;
  Spawn::Request::SharedPtr spawn_request = std::make_shared<Spawn::Request>();

public:
  SpawnTurtle() : Node("spawn_turtle_node"){
    spawn_client = this->create_client<Spawn>("spawn");
    
    while (!spawn_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
  }

  auto send_request(){
    get_user_input("> Turtle X position : ", spawn_request->x);
    get_user_input("> Turtle Y position : ", spawn_request->y);
    get_user_input("> Turtle Angle : ", spawn_request->theta);

    get_user_input("> Turtle Name : ", spawn_request->name);

    return spawn_client->async_send_request(spawn_request);
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpawnTurtle>();
  auto result = node->send_request();

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Turtle Named : %s Spawned Successfully.", result.get()->name.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();

  return 0;
}
