#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class BasicComponent : public rclcpp::Node
{
private:

  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  BasicComponent() : Node("BasicComponent")
  {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, std::bind(&BasicComponent::on_timer, this));
  }

  void on_timer(){
    RCLCPP_INFO(this->get_logger(), "Publishing: ");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<BasicComponent>());

  rclcpp::shutdown();
  return 0;
}