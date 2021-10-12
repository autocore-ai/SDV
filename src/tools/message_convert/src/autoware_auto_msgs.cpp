#include "message_convert/autoware_auto_msgs.hpp"
#include "message_convert/builtin_interfaces.hpp"
#include "message_convert/geometry_msgs.hpp"
#include "message_convert/std_msgs.hpp"
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <vector>
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

autoware_auto_msgs__msg__VehicleKinematicState Convert (autoware_auto_msgs::msg::VehicleKinematicState &source_msg) {
    autoware_auto_msgs__msg__VehicleKinematicState dest_msg;
    dest_msg.state = Convert(source_msg.state);
    dest_msg.header = Convert(source_msg.header);
    dest_msg.delta = Convert(source_msg.delta);
    return dest_msg;
}

autoware_auto_msgs::msg::VehicleKinematicState Convert (autoware_auto_msgs__msg__VehicleKinematicState &source_msg) {
    autoware_auto_msgs::msg::VehicleKinematicState dest_msg;
    dest_msg.state = Convert(source_msg.state);
    dest_msg.header = Convert(source_msg.header);
    dest_msg.delta = Convert(source_msg.delta);
    return dest_msg;
}

autoware_auto_msgs__msg__VehicleStateReport Convert (autoware_auto_msgs::msg::VehicleStateReport &source_msg) {
    autoware_auto_msgs__msg__VehicleStateReport dest_msg;
    dest_msg.stamp = Convert(source_msg.stamp);
    dest_msg.blinker = source_msg.blinker;
    dest_msg.fuel = source_msg.fuel;
    dest_msg.gear = source_msg.gear;
    dest_msg.hand_brake = source_msg.hand_brake;
    dest_msg.headlight = source_msg.headlight;
    dest_msg.horn = source_msg.horn;
    dest_msg.mode = source_msg.mode;
    dest_msg.wiper = source_msg.wiper;
    return dest_msg;
}

autoware_auto_msgs__msg__TrajectoryPoint Convert (autoware_auto_msgs::msg::TrajectoryPoint &source_msg) {
    autoware_auto_msgs__msg__TrajectoryPoint dest_msg;
    dest_msg.acceleration_mps2 = source_msg.acceleration_mps2;
    dest_msg.front_wheel_angle_rad = source_msg.front_wheel_angle_rad;
    dest_msg.heading = Convert(source_msg.heading);
    dest_msg.heading_rate_rps = source_msg.heading_rate_rps;
    dest_msg.lateral_velocity_mps = source_msg.lateral_velocity_mps;
    dest_msg.longitudinal_velocity_mps = source_msg.longitudinal_velocity_mps;
    dest_msg.rear_wheel_angle_rad = source_msg.rear_wheel_angle_rad;
    dest_msg.time_from_start = Convert(source_msg.time_from_start);
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    return dest_msg;
}

autoware_auto_msgs::msg::TrajectoryPoint Convert (autoware_auto_msgs__msg__TrajectoryPoint &source_msg) {
    autoware_auto_msgs::msg::TrajectoryPoint dest_msg;
    dest_msg.acceleration_mps2 = source_msg.acceleration_mps2;
    dest_msg.front_wheel_angle_rad = source_msg.front_wheel_angle_rad;
    dest_msg.heading = Convert(source_msg.heading);
    dest_msg.heading_rate_rps = source_msg.heading_rate_rps;
    dest_msg.lateral_velocity_mps = source_msg.lateral_velocity_mps;
    dest_msg.longitudinal_velocity_mps = source_msg.longitudinal_velocity_mps;
    dest_msg.rear_wheel_angle_rad = source_msg.rear_wheel_angle_rad;
    dest_msg.time_from_start = Convert(source_msg.time_from_start);
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    return dest_msg;
}

autoware_auto_msgs__msg__Complex32 Convert (autoware_auto_msgs::msg::Complex32 &source_msg) {
    autoware_auto_msgs__msg__Complex32 dest_msg;
    dest_msg.imag = source_msg.imag;
    dest_msg.real = source_msg.real;
    return dest_msg;
}

autoware_auto_msgs::msg::Complex32 Convert (autoware_auto_msgs__msg__Complex32 &source_msg) {
    autoware_auto_msgs::msg::Complex32 dest_msg;
    dest_msg.imag = source_msg.imag;
    dest_msg.real = source_msg.real;
    return dest_msg;
}

autoware_auto_msgs::msg::Trajectory Convert (autoware_auto_msgs__msg__Trajectory &source_msg) {
    autoware_auto_msgs::msg::Trajectory dest_msg;
    dest_msg.header = Convert(source_msg.header);
    dest_msg.points = rosidl_runtime_cpp::BoundedVector<autoware_auto_msgs::msg::TrajectoryPoint, 100>();
    std::vector<autoware_auto_msgs::msg::TrajectoryPoint> points;
    for (size_t i = 0; i < source_msg.points.size; i++) {
        dest_msg.points.push_back(Convert(source_msg.points.data[i]));
    }
    return dest_msg;
}

autoware_auto_msgs__msg__Trajectory Convert (autoware_auto_msgs::msg::Trajectory &source_msg) {
    autoware_auto_msgs__msg__Trajectory dest_msg;
    
    dest_msg.header = Convert(source_msg.header);
    std::vector<autoware_auto_msgs__msg__TrajectoryPoint> trajectory_points;
    for (size_t i = 0; i < source_msg.points.size(); i++) {
        trajectory_points.push_back(Convert(source_msg.points[i]));
    }

    dest_msg.points.data = trajectory_points.data();
    dest_msg.points.capacity = source_msg.points.capacity();
    dest_msg.points.size = source_msg.points.size();

    return dest_msg;
}
