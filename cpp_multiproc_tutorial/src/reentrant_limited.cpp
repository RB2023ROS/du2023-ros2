#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ReentrantNode : public rclcpp::Node{
private:
  rclcpp::TimerBase::SharedPtr timer_1;
  rclcpp::TimerBase::SharedPtr timer_2;
  rclcpp::TimerBase::SharedPtr timer_3;
  rclcpp::CallbackGroup::SharedPtr cb_group_1;
  rclcpp::CallbackGroup::SharedPtr cb_group_2;
  rclcpp::CallbackGroup::SharedPtr cb_group_3;

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

  void timer_cb_3(){
    RCLCPP_INFO(this->get_logger(), "timer_cb_3");
    
    // assume that something logic happens here
    std::this_thread::sleep_for(1s);
  }

public:
  ReentrantNode() : Node("deadlock_example"){

    cb_group_1 = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant
    );

    cb_group_2 = cb_group_1;

    cb_group_3 = cb_group_1;

    timer_1 = this->create_wall_timer(
      200ms, std::bind(&ReentrantNode::timer_cb_1, this), cb_group_1
    );

    timer_2 = this->create_wall_timer(
      200ms, std::bind(&ReentrantNode::timer_cb_2, this), cb_group_2
    );

    timer_3 = this->create_wall_timer(
      200ms, std::bind(&ReentrantNode::timer_cb_3, this), cb_group_3
    );
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "Max CPU Core : " << std::thread::hardware_concurrency() << std::endl;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 1);
  // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 3);
  auto test_node = std::make_shared<ReentrantNode>();
  
  executor.add_node(test_node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}