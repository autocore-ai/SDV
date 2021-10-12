#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3.h>

__attribute__((visibility("default"))) geometry_msgs::msg::PoseWithCovarianceStamped Convert (geometry_msgs__msg__PoseWithCovarianceStamped &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::PoseWithCovariance Convert (geometry_msgs__msg__PoseWithCovariance &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Quaternion Convert (geometry_msgs__msg__Quaternion &source_msg);
__attribute__((visibility("default"))) geometry_msgs__msg__Quaternion Convert (geometry_msgs::msg::Quaternion &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Point Convert (geometry_msgs__msg__Point &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Pose Convert (geometry_msgs__msg__Pose &source_msg);

__attribute__((visibility("default"))) geometry_msgs__msg__Transform Convert(geometry_msgs::msg::Transform &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Transform Convert(geometry_msgs__msg__Transform &source_msg);

__attribute__((visibility("default"))) geometry_msgs__msg__Vector3 Convert(geometry_msgs::msg::Vector3 &source_msg);
__attribute__((visibility("default"))) geometry_msgs::msg::Vector3 Convert(geometry_msgs__msg__Vector3 &source_msg);