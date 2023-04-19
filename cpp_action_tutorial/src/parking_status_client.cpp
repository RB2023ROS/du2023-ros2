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
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/parking.hpp"

using namespace std::placeholders;
using Parking = custom_interfaces::action::Parking;
using GoalHandleParking = rclcpp_action::ClientGoalHandle<Parking>;

class ParkingActionClient : public rclcpp::Node {
private:
  rclcpp_action::Client<Parking>::SharedPtr m_action_client;

public:
  ParkingActionClient() : Node("parking_action_client"){
    m_action_client = rclcpp_action::create_client<Parking>(this, "src_parking");
  }

  void send_goal(){

    if (!m_action_client->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Parking::Goal();
    goal_msg.start_flag = true;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Parking>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&ParkingActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ParkingActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ParkingActionClient::result_callback, this, _1);
    
    m_action_client->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(std::shared_future<GoalHandleParking::SharedPtr> future){
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleParking::SharedPtr,
    const std::shared_ptr<const Parking::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->distance);

    if(feedback->distance > 6.0)
      m_action_client->async_cancel_all_goals();
  }

  void result_callback(const GoalHandleParking::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->message.c_str());
    rclcpp::shutdown();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<ParkingActionClient>();
  client_node->send_goal();
  rclcpp::spin(client_node);
  
  rclcpp::shutdown();
  return 0;
}