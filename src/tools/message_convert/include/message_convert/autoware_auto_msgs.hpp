#pragma once
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include "message_convert/rust_msgs.hpp"

__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleStateCommand Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand Convert (autoware_auto_msgs::msg::VehicleStateCommand &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleControlCommand Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleControlCommand &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleControlCommand Convert (autoware_auto_msgs::msg::VehicleControlCommand &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState Convert (autoware_auto_msgs::msg::VehicleKinematicState &source_msg);
__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleKinematicState Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport Convert (autoware_auto_msgs::msg::VehicleStateReport &source_msg);
__attribute__((visibility("default"))) autoware_auto_msgs::msg::VehicleStateReport Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport  &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint Convert (autoware_auto_msgs::msg::TrajectoryPoint &source_msg);
__attribute__((visibility("default"))) autoware_auto_msgs::msg::TrajectoryPoint Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 Convert (autoware_auto_msgs::msg::Complex32 &source_msg);
__attribute__((visibility("default"))) autoware_auto_msgs::msg::Complex32 Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::Trajectory Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory Convert (autoware_auto_msgs::msg::Trajectory &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute Convert (autoware_auto_msgs::msg::HADMapRoute &source_msg);
__attribute__((visibility("default"))) autoware_auto_msgs::msg::HADMapRoute Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::RoutePoint Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint Convert (autoware_auto_msgs::msg::RoutePoint &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::HADMapSegment Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment Convert (autoware_auto_msgs::msg::HADMapSegment &source_msg);

__attribute__((visibility("default"))) autoware_auto_msgs::msg::MapPrimitive Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_MapPrimitive  &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::autoware_auto_msgs_MapPrimitive Convert (autoware_auto_msgs::msg::MapPrimitive &source_msg);
