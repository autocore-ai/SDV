#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include "message_convert/rust_msgs.hpp"
__attribute__((visibility("default"))) builtin_interfaces::msg::Time Convert (zenoh_flow::autoware_auto::builtin_interfaces_Time &source_msg);
__attribute__((visibility("default"))) zenoh_flow::autoware_auto::builtin_interfaces_Time Convert (builtin_interfaces::msg::Time &source_msg);

__attribute__((visibility("default"))) zenoh_flow::autoware_auto::builtin_interfaces_Duration Convert(builtin_interfaces::msg::Duration &source_msg);
__attribute__((visibility("default"))) builtin_interfaces::msg::Duration Convert(zenoh_flow::autoware_auto::builtin_interfaces_Duration &source_msg);