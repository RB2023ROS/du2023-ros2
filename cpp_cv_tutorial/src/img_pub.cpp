#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
const std::string VIDEO_FILE_ROOT = "/home/kimsooyoung/ros2_ws/go1_lidar.mp4";

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;

class ImagePublisher : public rclcpp::Node {
private:
  cv::Mat my_img;
  cv::VideoCapture cap;
  Image::SharedPtr img_msg;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Image>::SharedPtr image_pub;

public:
  ImagePublisher() : Node("cv_img_pub"){
    image_pub = this->create_publisher<Image>("video_frames", 10);

    timer_ = this->create_wall_timer(
        30ms, std::bind(&ImagePublisher::timer_callback, this)
    );

    cap.open(VIDEO_FILE_ROOT);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
    }
  }
  void timer_callback() {
    // // Create a new 640x480 image
    // cv::Mat my_image(cv::Size(640, 480), CV_8UC3);
 
    //--- GRAB AND WRITE LOOP
    cap.read(my_img);
    if (my_img.empty()){
      std::cerr << "ERROR! blank frame grabbed\n";
      rclcpp::shutdown();
    }

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_img).toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    image_pub->publish(*img_msg.get());
  }
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<ImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
