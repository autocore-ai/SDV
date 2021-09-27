#include "message_convert/std_msgs.hpp"
#include "message_convert/builtin_interfaces.hpp"

std_msgs::msg::Header Convert (std_msgs__msg__Header &source_msg) {
    std_msgs::msg::Header dest_msg;
    dest_msg.frame_id = std::string(source_msg.frame_id.data, source_msg.frame_id.size);
    dest_msg.stamp = Convert(source_msg.stamp);
    return dest_msg;
}

std_msgs__msg__Header Convert (std_msgs::msg::Header &source_msg) {
    std_msgs__msg__Header dest_msg;
    dest_msg.frame_id.capacity = source_msg.frame_id.capacity();
    dest_msg.frame_id.data = const_cast<char*>(source_msg.frame_id.data());
    dest_msg.frame_id.size = source_msg.frame_id.length();
    dest_msg.stamp = Convert(source_msg.stamp);
    return dest_msg;
}
