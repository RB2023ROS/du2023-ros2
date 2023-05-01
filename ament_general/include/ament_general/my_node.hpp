#ifndef AMENT_GENERAL__MY_NODE_HPP_
#define AMENT_GENERAL__MY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyNode: public rclcpp::Node {
private:
  size_t count;
  rclcpp::TimerBase::SharedPtr timer;

public:
  MyNode();
  void timer_callback();
};

#endif