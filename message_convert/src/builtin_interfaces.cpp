#include "message_convert/builtin_interfaces.hpp"
builtin_interfaces::msg::Time Convert (builtin_interfaces__msg__Time &source_msg) {
    builtin_interfaces::msg::Time dest_msg;
    dest_msg.nanosec = source_msg.nanosec;
    dest_msg.sec = source_msg.sec;
    return dest_msg;
}
