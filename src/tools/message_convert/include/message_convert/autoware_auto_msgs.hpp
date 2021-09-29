#pragma once
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.h>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.h>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.h>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.h>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/complex32.h>

__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleStateCommand Convert (autoware_auto_msgs__msg__VehicleStateCommand &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleControlCommand Convert (autoware_auto_msgs__msg__VehicleControlCommand &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs__msg__VehicleKinematicState Convert (autoware_auto_msgs::msg::VehicleKinematicState &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs__msg__VehicleStateReport Convert (autoware_auto_msgs::msg::VehicleStateReport &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs__msg__TrajectoryPoint Convert (autoware_auto_msgs::msg::TrajectoryPoint &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs__msg__Complex32 Convert (autoware_auto_msgs::msg::Complex32 &source_msg);