// Copyright 2023 @RoadBalance
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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/parking.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using Parking = custom_interfaces::action::Parking;
using GoalHandleParking = rclcpp_action::ServerGoalHandle<Parking>;

class ParkingActionServer : public rclcpp::Node {
private:
  rclcpp_action::Server<Parking>::SharedPtr m_action_server;
  rclcpp::Subscription<LaserScan>::SharedPtr m_laser_sub;
  
  bool is_sub = false;
  bool is_done = false;
  double f_obs_distance = 5.0;
  double r_obs_distance = 5.0;
  double l_obs_distance = 5.0;

public:
  ParkingActionServer() : Node("parking_action_server") {
    using namespace std::placeholders;
    // Create an action server with three callbacks
    //   'handle_goal' and 'handle_cancel' are called by the Executor
    //   (rclcpp::spin) 'execute' is called whenever 'handle_goal' returns by
    //   accepting a goal
    //    Calls to 'execute' are made in an available thread from a pool of
    //    four.
    m_action_server = rclcpp_action::create_server<Parking>(
      this, "src_parking",
      std::bind(&ParkingActionServer::handle_goal, this, _1, _2),
      std::bind(&ParkingActionServer::handle_cancel, this, _1),
      std::bind(&ParkingActionServer::handle_accepted, this, _1)
    );

    m_laser_sub = this->create_subscription<LaserScan>(
      "scan", 10, std::bind(&ParkingActionServer::laser_callback, this, _1)
    );

    RCLCPP_INFO(get_logger(), "Action Ready...");
  }

  void laser_callback(const LaserScan::ConstSharedPtr msg) {
    if(is_sub){
      f_obs_distance = msg->ranges[60];
      r_obs_distance = msg->ranges[30];
      l_obs_distance = msg->ranges[90];
      
      if(!is_done)
        RCLCPP_INFO(get_logger(), "sub success");
    }
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Parking::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Got goal request with order %s", goal->start_flag ? "true" : "false");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleParking> goal_handle) {
    RCLCPP_WARN(get_logger(), "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleParking> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    using namespace std::placeholders;
    std::thread{std::bind(&ParkingActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleParking> goal_handle) {

    RCLCPP_INFO(get_logger(), "Executing goal");

    is_sub = true;
    rclcpp::WallRate loop_rate(1);  // 1 Hz

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Parking::Feedback>();
    auto result = std::make_shared<Parking::Result>();

    while (f_obs_distance > 0.5 && is_done == false) {
      if (goal_handle->is_canceling()) {
        result->message = "Canceled";
        goal_handle->canceled(result);
        is_done = true;
        RCLCPP_WARN(get_logger(), "Goal Canceled");
        return;
      }
      
      feedback->distance = f_obs_distance;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Distance from forward obstacle %f", f_obs_distance);
      
      loop_rate.sleep();
    }

    // Check if there is an obstacle on the left or right
    auto lr_diff = abs(r_obs_distance - l_obs_distance) < 0.15 ? true : false;

    if (lr_diff)
      result->message = "[Success!] Oh... Teach me how you did :0";
    else
      result->message = "[Fail] Be careful, Poor Driver! ";

    goal_handle->succeed(result);
    is_done = true;
    RCLCPP_INFO(get_logger(), "Goal Succeeded");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto server_node = std::make_shared<ParkingActionServer>();
  
  executor.add_node(server_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}