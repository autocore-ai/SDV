#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "message_convert/rust_msgs.hpp"

__attribute__((visibility("default"))) geometry_msgs::msg::PoseWithCovarianceStamped Convert (zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovarianceStamped &source_msg);

__attribute__((visibility("default"))) geometry_msgs::msg::PoseWithCovariance Convert (zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovariance &source_msg);

__attribute__((visibility("default"))) geometry_msgs::msg::Quaternion Convert (zenoh_flow::autoware_auto::geometry_msgs_Quaternion &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::geometry_msgs_Quaternion Convert (geometry_msgs::msg::Quaternion &source_msg);

__attribute__((visibility("default"))) geometry_msgs::msg::Point Convert (zenoh_flow::autoware_auto::geometry_msgs_Point &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::geometry_msgs_Point Convert (geometry_msgs::msg::Point &source_msg);

__attribute__((visibility("default"))) geometry_msgs::msg::Pose Convert (zenoh_flow::autoware_auto::geometry_msgs_Pose &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::geometry_msgs_Transform Convert(geometry_msgs::msg::Transform &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Transform Convert(zenoh_flow::autoware_auto::geometry_msgs_Transform &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::geometry_msgs_Vector3 Convert(geometry_msgs::msg::Vector3 &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Vector3 Convert(zenoh_flow::autoware_auto::geometry_msgs_Vector3 &source_msg);