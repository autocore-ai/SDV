#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/action/rotate_absolute.hpp>

class TurtleSimSource
{
public:
  TurtleSimSource();
  int GetKey();
  int keyLoop();

private:
  void spin();
  void sendGoal(float theta);
  void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future);
  void cancelGoal();
  
  char c;
  bool dirty = false;
  rclcpp::Node::SharedPtr nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
  rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr goal_handle_;
};

int initialize();
