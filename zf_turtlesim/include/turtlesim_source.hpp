#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <zf_turtlesim_msgs/action/rotate_absolute.hpp>

class TurtleSimSource
{
public:
  TurtleSimSource(int argc, char **argv);
  ~TurtleSimSource();
  int KeyLoop();
  int Run();
  int Quit();

  geometry_msgs::msg::Twist twist;

private:
  void spin();
  void sendGoal(float theta);
  void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<zf_turtlesim_msgs::action::RotateAbsolute>::SharedPtr> future);
  void cancelGoal();
  void updateTwist();
  int getKey();

  char c;
  bool dirty = false;

  rclcpp::Node::SharedPtr nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_action::Client<zf_turtlesim_msgs::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
  rclcpp_action::ClientGoalHandle<zf_turtlesim_msgs::action::RotateAbsolute>::SharedPtr goal_handle_;
};
