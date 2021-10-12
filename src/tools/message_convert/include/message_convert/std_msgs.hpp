#pragma once
#include <std_msgs/msg/header.hpp>

#include "message_convert/rust_msgs.hpp"

__attribute__((visibility("default"))) std_msgs::msg::Header Convert (zenoh_flow::autoware_auto::std_msgs_Header &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::std_msgs_Header Convert (std_msgs::msg::Header &source_msg);