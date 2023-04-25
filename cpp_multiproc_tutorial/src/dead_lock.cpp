#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DeadLockNode : public rclcpp::Node{
private:
  rclcpp::TimerBase::SharedPtr timer_1;
  rclcpp::TimerBase::SharedPtr timer_2;


  void timer_cb_1(){
    RCLCPP_INFO(this->get_logger(), "timer_cb_1");
    
    // assume that something logic happens here
    std::this_thread::sleep_for(5s);
  }

  void timer_cb_2(){
    RCLCPP_INFO(this->get_logger(), "timer_cb_2");
    
    // assume that something logic happens here
    std::this_thread::sleep_for(2s);
  }

public:
  DeadLockNode() : Node("deadlock_example"){

    timer_1 = this->create_wall_timer(
      200ms, std::bind(&DeadLockNode::timer_cb_1, this)
    );

    timer_2 = this->create_wall_timer(
      200ms, std::bind(&DeadLockNode::timer_cb_2, this)
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DeadLockNode>());

  rclcpp::shutdown();

  return 0;
}