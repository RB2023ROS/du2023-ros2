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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using SetBool = example_interfaces::srv::SetBool;
using Image   = sensor_msgs::msg::Image;

/**
 * std_srvs::srv::SetBool Description
 * 
 * bool data # e.g. for hardware enabling / disabling
 * --- 
 * bool success   # indicate successful run of triggered service 
 * string message # informational, e.g. for error messages
 */

class PictureNode : public rclcpp::Node {
private:
  bool is_request = false;
  std::string img_topic_name = "/rgb_cam/rgb_cam/image_raw";

  rclcpp::Service<SetBool>::SharedPtr bool_server;
  rclcpp::Subscription<Image>::SharedPtr image_subscriber;

  void img_sub_callback(const Image::ConstSharedPtr msg){
    if (this->is_request){
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
      // cv::Mat matFromImage(const sensor_msgs::msg::Image & source)

      std::string file_name = std::to_string(this->get_clock()->now().seconds()) + ".jpg";
      cv::imwrite(file_name, img);

      RCLCPP_INFO(this->get_logger(), "Image saved as %s", file_name.c_str());
      this->is_request = false;
    }
  }

  void server_callback(const std::shared_ptr<SetBool::Request> request,
                      const std::shared_ptr<SetBool::Response> response){
    if (request->data) {
      RCLCPP_INFO(this->get_logger(), "KimChi~");
      this->is_request = true;
    }

    response->success = true;
    response->message = "Successfully image written";
  }

public:
  PictureNode() : Node("turtle_circle_server"){
    image_subscriber = this->create_subscription<Image>(
      this->img_topic_name, 10,
      std::bind(&PictureNode::img_sub_callback, this, std::placeholders::_1)
    );

    bool_server = this->create_service<SetBool>(
      "take_picture",
      std::bind(&PictureNode::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Taking Picture Server Started, Waiting for Request...");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PictureNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
