#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class TF2Listener : public rclcpp::Node
{
private:
  // Bozolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
  
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::string target_frame_;

public:
  TF2Listener(): Node("tf2_frame_listener"){
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Create turtle2 velocity publisher
    m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("sphere_vel", 10);

    // Call timer_cb function every 100ms
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TF2Listener::timer_cb, this));
  }

  void timer_cb(){
    // Store frame names in variables that will be used to
    // compute transformations

    TransformStamped t;

    try {
      t = m_tf_buffer->lookupTransform(
        "animated_sphere", "world", tf2::TimePointZero
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO( 
        this->get_logger(), "Could not transform %s to %s: %s",
        "animated_sphere", "world", ex.what()
      );
      return;
    }

    Twist msg;

    static const double scaleRotationRate = 1.0;
    msg.angular.z = scaleRotationRate * atan2(
      t.transform.translation.y,
      t.transform.translation.x);

    static const double scaleForwardSpeed = 1.0;
    msg.linear.x = scaleForwardSpeed * sqrt(
      pow(t.transform.translation.x, 2) +
      pow(t.transform.translation.y, 2));

    RCLCPP_INFO( 
      this->get_logger(), "linear vel : %f, angular vel : %f",
      msg.linear.x, msg.angular.z
    );

    m_publisher->publish(msg);
  }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TF2Listener>());
  
  rclcpp::shutdown();
  return 0;
}