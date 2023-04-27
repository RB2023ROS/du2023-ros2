#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace composition_tutorial {

class BasicComponent : public rclcpp::Node {
public:
  // COMPOSITION_PUBLIC
  explicit BasicComponent(const rclcpp::NodeOptions & options)
  : Node("BasicComponent", options), count_(0){
    // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(1s, std::bind(&BasicComponent::on_timer, this));
  }

protected:
  void on_timer(){
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    std::flush(std::cout);

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg));
  }

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition_tutorial::BasicComponent)