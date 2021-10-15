#include "message_convert/autoware_auto_msgs.hpp"
#include "message_convert/builtin_interfaces.hpp"
#include "message_convert/geometry_msgs.hpp"
#include "message_convert/std_msgs.hpp"
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <vector>
autoware_auto_msgs::msg::VehicleStateCommand Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand &source_msg) {
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

zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand Convert (autoware_auto_msgs::msg::VehicleStateCommand &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand dest_msg;
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

autoware_auto_msgs::msg::VehicleControlCommand Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleControlCommand &source_msg) {
    autoware_auto_msgs::msg::VehicleControlCommand dest_msg;
    dest_msg.stamp = Convert(source_msg.stamp);
    dest_msg.front_wheel_angle_rad = source_msg.front_wheel_angle_rad;
    dest_msg.rear_wheel_angle_rad = source_msg.rear_wheel_angle_rad;
    dest_msg.long_accel_mps2 = source_msg.long_accel_mps2;
    dest_msg.velocity_mps = source_msg.velocity_mps;
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState Convert (autoware_auto_msgs::msg::VehicleKinematicState &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState dest_msg;
    dest_msg.state = Convert(source_msg.state);
    dest_msg.header = Convert(source_msg.header);
    dest_msg.delta = Convert(source_msg.delta);
    return dest_msg;
}

autoware_auto_msgs::msg::VehicleKinematicState Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState &source_msg) {
    autoware_auto_msgs::msg::VehicleKinematicState dest_msg;
    dest_msg.state = Convert(source_msg.state);
    dest_msg.header = Convert(source_msg.header);
    dest_msg.delta = Convert(source_msg.delta);
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport Convert (autoware_auto_msgs::msg::VehicleStateReport &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport dest_msg;
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

autoware_auto_msgs::msg::VehicleStateReport Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport  &source_msg) {
    autoware_auto_msgs::msg::VehicleStateReport dest_msg;
    dest_msg.blinker = source_msg.blinker;
    dest_msg.fuel = source_msg.fuel;
    dest_msg.gear = source_msg.gear;
    dest_msg.hand_brake = source_msg.hand_brake;
    dest_msg.headlight = source_msg.headlight;
    dest_msg.horn = source_msg.horn;
    dest_msg.mode = source_msg.mode;
    dest_msg.stamp = Convert(source_msg.stamp);
    dest_msg.wiper = source_msg.wiper;
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint Convert (autoware_auto_msgs::msg::TrajectoryPoint &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint dest_msg;
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

autoware_auto_msgs::msg::TrajectoryPoint Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint &source_msg) {
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

zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 Convert (autoware_auto_msgs::msg::Complex32 &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 dest_msg;
    dest_msg.imag = source_msg.imag;
    dest_msg.real = source_msg.real;
    return dest_msg;
}

autoware_auto_msgs::msg::Complex32 Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 &source_msg) {
    autoware_auto_msgs::msg::Complex32 dest_msg;
    dest_msg.imag = source_msg.imag;
    dest_msg.real = source_msg.real;
    return dest_msg;
}

autoware_auto_msgs::msg::Trajectory Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory &source_msg) {
    autoware_auto_msgs::msg::Trajectory dest_msg;
    dest_msg.header = Convert(source_msg.header);
    dest_msg.points = rosidl_runtime_cpp::BoundedVector<autoware_auto_msgs::msg::TrajectoryPoint, 100>();
    for (size_t i = 0; i < source_msg.points.size(); i++) {
        dest_msg.points.push_back(Convert(source_msg.points.data()[i]));
    }
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory Convert (autoware_auto_msgs::msg::Trajectory &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory dest_msg;
    
    dest_msg.header = Convert(source_msg.header);
     rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint> trajectory_points;
    for (size_t i = 0; i < source_msg.points.size(); i++) {
        trajectory_points.push_back(Convert(source_msg.points[i]));
    }

    dest_msg.points = trajectory_points;
    return dest_msg;
}


zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute Covert (autoware_auto_msgs::msg::HADMapRoute &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute dest_msg;
    dest_msg.header = Convert(source_msg.header);
    dest_msg.goal_point = Convert(source_msg.goal_point);
    rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment> segments;
    for (size_t i = 0; i < source_msg.segments.size(); i++) {
        segments.push_back(Convert(source_msg.segments[i]));
    }
    dest_msg.segments = segments;
    dest_msg.start_point = Convert(source_msg.start_point);
    return dest_msg;
}

autoware_auto_msgs::msg::RoutePoint Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint &source_msg) {
    autoware_auto_msgs::msg::RoutePoint dest_msg;
    dest_msg.heading = Convert(source_msg.heading);
    dest_msg.position = Convert(source_msg.position);
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint Convert (autoware_auto_msgs::msg::RoutePoint &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint dest_msg;
    dest_msg.heading = Convert(source_msg.heading);
    dest_msg.position = Convert(source_msg.position);
    return dest_msg;
}

autoware_auto_msgs::msg::HADMapSegment Convert (zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment &source_msg) {
    autoware_auto_msgs::msg::HADMapSegment dest_msg;
    dest_msg.preferred_primitive_id = source_msg.preferred_primitive_id;
    for (size_t i = 0; i < source_msg.primitives.size(); i++) {
        dest_msg.primitives.push_back(Convert(source_msg.primitives[i]));
    }
    return dest_msg;
}

zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment Convert (autoware_auto_msgs::msg::HADMapSegment &source_msg) {
    zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment dest_msg;
    dest_msg.preferred_primitive_id = source_msg.preferred_primitive_id;
    for (size_t i = 0; i < source_msg.primitives.size(); i++) {
        dest_msg.primitives.push_back(Convert(source_msg.primitives[i]));
    }
    return dest_msg;
}
