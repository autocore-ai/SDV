#pragma once
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.h>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.h>

autoware_auto_msgs::msg::VehicleStateCommand Convert (autoware_auto_msgs__msg__VehicleStateCommand &source_msg);
autoware_auto_msgs::msg::VehicleControlCommand Convert (autoware_auto_msgs__msg__VehicleControlCommand &source_msg);