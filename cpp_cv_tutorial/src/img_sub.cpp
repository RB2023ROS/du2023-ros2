#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using Image = sensor_msgs::msg::Image;

// Fill in your image processing function
cv::Mat image_processing(const cv::Mat img_in){
  
  // Create output image
  cv::Mat img_out;
  cv::Mat img_small;
  
  // Processing
  img_out = img_in;

  // Show image in a different window
  cv::resize(img_in, img_small, cv::Size(640, 480));
  cv::imshow("img_out", img_small);
  cv::waitKey(3);

  // You must to return a 3-channels image to show it in ROS, so do it with 1-channel images
  // cv::cvtColor(img_out, img_out, cv::COLOR_GRAY2BGR);
  return img_out;
}

class ImageSubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<Image>::SharedPtr m_img_sub;
  rclcpp::Publisher<Image>::SharedPtr m_img_pub;
public:
  ImageSubscriber(): Node("image_subscriber"){

    std::string topic_name = "video_frames";

    m_img_sub = this->create_subscription<Image>(
      topic_name, 10, std::bind(&ImageSubscriber::image_cb, this, std::placeholders::_1)
    );
  }

  void image_cb(const Image::SharedPtr msg) const{     
    // Convert ROS Image to CV Image
    cv::Mat frame = cv_bridge::toCvCopy(
      msg, sensor_msgs::image_encodings::BGR8
    )->image;

    // Image processing
    cv::Mat proced_img = image_processing(frame);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}