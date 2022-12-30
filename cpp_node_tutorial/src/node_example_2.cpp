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

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

static int count = 0;

void timer_callback(){
  std::cout << "==== Hello ROS 2 : " << count << " ====" << std::endl;
  count++;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_node_2");
  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), timer_callback);

  rclcpp::spin(node); 

  rclcpp::shutdown();
  return 0;
}
