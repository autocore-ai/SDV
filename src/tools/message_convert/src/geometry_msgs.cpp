#include "message_convert/geometry_msgs.hpp"
#include "message_convert/std_msgs.hpp"

geometry_msgs::msg::PoseWithCovarianceStamped Convert (zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovarianceStamped &source_msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped dest_msg;
    dest_msg.header = Convert(source_msg.header);
    dest_msg.pose = Convert(source_msg.pose);
    return dest_msg;
}

geometry_msgs::msg::PoseWithCovariance Convert (zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovariance &source_msg) {
    geometry_msgs::msg::PoseWithCovariance dest_msg;
    dest_msg.pose = Convert(source_msg.pose);
    std::copy(std::begin(source_msg.covariance), std::end(source_msg.covariance), dest_msg.covariance.begin());
    return dest_msg;
}

geometry_msgs::msg::Quaternion Convert (zenoh_flow::autoware_auto::geometry_msgs_Quaternion &source_msg) {
    geometry_msgs::msg::Quaternion dest_msg;
    dest_msg.w = source_msg.w;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}
zenoh_flow::autoware_auto::geometry_msgs_Quaternion Convert (geometry_msgs::msg::Quaternion &source_msg) {
    zenoh_flow::autoware_auto::geometry_msgs_Quaternion dest_msg;
    dest_msg.w = source_msg.w;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}

geometry_msgs::msg::Point Convert (zenoh_flow::autoware_auto::geometry_msgs_Point &source_msg) {
    geometry_msgs::msg::Point dest_msg;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}

zenoh_flow::autoware_auto::geometry_msgs_Point Convert (geometry_msgs::msg::Point &source_msg) {
    zenoh_flow::autoware_auto::geometry_msgs_Point dest_msg;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}

geometry_msgs::msg::Pose Convert (zenoh_flow::autoware_auto::geometry_msgs_Pose &source_msg) {
    geometry_msgs::msg::Pose dest_msg;
    dest_msg.orientation = Convert(source_msg.orientation);
    dest_msg.position = Convert(source_msg.position);
    return dest_msg;
}

zenoh_flow::autoware_auto::geometry_msgs_Transform Convert(geometry_msgs::msg::Transform &source_msg) {
    zenoh_flow::autoware_auto::geometry_msgs_Transform dest_msg;
    dest_msg.rotation = Convert(source_msg.rotation);
    dest_msg.translation = Convert(source_msg.translation);
    return dest_msg;
}

geometry_msgs::msg::Transform Convert (zenoh_flow::autoware_auto::geometry_msgs_Transform &source_msg) {
    geometry_msgs::msg::Transform dest_msg;
    dest_msg.rotation = Convert(source_msg.rotation);
    dest_msg.translation = Convert(source_msg.translation);
    return dest_msg;
}

zenoh_flow::autoware_auto::geometry_msgs_Vector3 Convert(geometry_msgs::msg::Vector3 &source_msg) {
    zenoh_flow::autoware_auto::geometry_msgs_Vector3 dest_msg;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}

geometry_msgs::msg::Vector3 Convert(zenoh_flow::autoware_auto::geometry_msgs_Vector3 &source_msg) {
    geometry_msgs::msg::Vector3 dest_msg;
    dest_msg.x = source_msg.x;
    dest_msg.y = source_msg.y;
    dest_msg.z = source_msg.z;
    return dest_msg;
}