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
#include <unistd.h>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

using Spawn = gazebo_msgs::srv::SpawnEntity;
using Empty = std_srvs::srv::Empty;

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

std::map<std::string, std::string> model_map = {
  {"1", "arm_part"},
  {"2", "beer"},
  {"3", "biscuits"},
  {"4", "book"},
  {"5", "bowl"},
  {"6", "create"},
  {"7", "disk_part"},
  {"8", "eraser"},
  {"9", "glue"},
  {"10", "hammer"},
  {"11", "plastic_cup"},
  {"12", "snacks"},
  {"13", "soap"},
  {"14", "soap2"},
  {"15", "soda_can"},
  {"16", "sticky_notes"}
};

class GZSpawnClient : public rclcpp::Node {
private:
  rclcpp::Client<Spawn>::SharedPtr spawn_client;
  rclcpp::Client<Empty>::SharedPtr pause_client;

  Spawn::Request::SharedPtr spawn_request = std::make_shared<Spawn::Request>();
  Empty::Request::SharedPtr pause_request = std::make_shared<Empty::Request>();
  std::string model_dir_path;

public:
  GZSpawnClient() : Node("gazebo_model_spawner"){
    char tmp[256];
    getcwd(tmp, 256);
    model_dir_path = std::string(tmp) + "/src/gz_ros2_examples/rgbd_world/models";
    
    pause_client = this->create_client<Empty>("pause_physics");
    spawn_client = this->create_client<Spawn>("spawn_entity");
    
    while (!pause_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "Pause service not available, waiting again...");
    }

    while (!spawn_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "Spawn service not available, waiting again...");
    }
  }

  auto send_pause_request(){
    return pause_client->async_send_request(pause_request);
  }

  auto send_spawn_request(const std::string &model_name){
    auto item = model_map.find(model_name);
    std::string model_path;
    
    if (item != model_map.end()) {
      spawn_request->name = item->second;
      model_path = model_dir_path + "/" + item->second + "/model.sdf";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Model Does not exist!");
      exit(0);
    }

    std::ifstream file(model_path);
    std::stringstream ss;
    ss << file.rdbuf();

    spawn_request->xml  = ss.str();
    spawn_request->initial_pose.position.x = 1.0;
    spawn_request->initial_pose.position.y = 0.0;
    spawn_request->initial_pose.position.z = 1.0;

    RCLCPP_INFO(this->get_logger(), "==== Sending service request to `/spawn_entity` ====");

    return spawn_client->async_send_request(spawn_request);
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GZSpawnClient>();

  auto pause_result = node->send_pause_request();
  // Wait for the spawn_result.
  if (rclcpp::spin_until_future_complete(node, pause_result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Pause Successfully Done.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to pause physics!");
  }

  std::string model_name;
  get_user_input(
    "> Enter Model name Among Below List\n"
    "1.arm_part\t2.beer         \t3.biscuits\n"
    "4.book     \t5.bowl        \t6.create\n"
    "7.disk_part\t8.eraser      \t9.glue\n"
    "10.hammer  \t11.plastic_cup\t12.snacks\n"
    "13.soap    \t14.soap2      \t15.soda_can\n"
    "16.sticky_notes\n"
    "[Type your choice]: " , model_name);

  auto spawn_result = node->send_spawn_request(model_name);
  // Wait for the spawn_result.
  if (rclcpp::spin_until_future_complete(node, spawn_result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Spawn Successfully Done.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to spawn model!");
  }

  rclcpp::shutdown();

  return 0;
}
