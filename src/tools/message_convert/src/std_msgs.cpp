#include "message_convert/std_msgs.hpp"
#include "message_convert/builtin_interfaces.hpp"

std_msgs::msg::Header Convert (zenoh_flow::autoware_auto::std_msgs_Header &source_msg) {
    std_msgs::msg::Header dest_msg;
    dest_msg.frame_id = std::string(source_msg.frame_id.data(), source_msg.frame_id.size());
    dest_msg.stamp = Convert(source_msg.stamp);
    return dest_msg;
}

zenoh_flow::autoware_auto::std_msgs_Header Convert (std_msgs::msg::Header &source_msg) {
    zenoh_flow::autoware_auto::std_msgs_Header dest_msg;
    dest_msg.frame_id = rust::String(source_msg.frame_id.data(), source_msg.frame_id.size());
    dest_msg.stamp = Convert(source_msg.stamp);
    return dest_msg;
}
