#include "message_convert/autoware_auto_msgs.hpp"
#include "message_convert/builtin_interfaces.hpp"

autoware_auto_msgs::msg::VehicleStateCommand Convert (autoware_auto_msgs__msg__VehicleStateCommand &source_msg) {
    autoware_auto_msgs::msg::VehicleStateCommand dest_msg;
    dest_msg.stamp = Convert(source_msg.stamp);
    dest_msg.blinker = source_msg.blinker;
    dest_msg.gear = source_msg.gear;
    dest_msg.hand_brake = source_msg.hand_brake;
    dest_msg.headlight = source_msg.headlight;
    dest_msg.horn = source_msg.horn;
    dest_msg.mode = source_msg.mode;
    dest_msg.wiper = source_msg.wiper;
    return dest_msg;
}
autoware_auto_msgs::msg::VehicleControlCommand Convert (autoware_auto_msgs__msg__VehicleControlCommand &source_msg) {
    autoware_auto_msgs::msg::VehicleControlCommand dest_msg;
    dest_msg.stamp = Convert(source_msg.stamp);
    dest_msg.front_wheel_angle_rad = source_msg.front_wheel_angle_rad;
    dest_msg.rear_wheel_angle_rad = source_msg.rear_wheel_angle_rad;
    dest_msg.long_accel_mps2 = source_msg.long_accel_mps2;
    dest_msg.velocity_mps = source_msg.velocity_mps;
    return dest_msg;
}
