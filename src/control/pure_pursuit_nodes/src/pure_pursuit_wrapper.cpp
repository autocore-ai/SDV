#include "pure_pursuit_nodes/pure_pursuit_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <message_convert/autoware_auto_msgs.hpp>
PurePursuitNodeWrapper::PurePursuitNodeWrapper() {
    rclcpp::init(0, nullptr);
    m_pure_pursuit_node_ptr = std::make_shared<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode>("pure_pursuit_node");
}

autoware_auto_msgs__msg__VehicleControlCommand PurePursuitNodeWrapper::updateCommand(autoware_auto_msgs__msg__Trajectory trajectory, 
    autoware_auto_msgs__msg__VehicleKinematicState vehicle_kinematic_state) {
    
    // 获得最新的消息
    rclcpp::spin_some(this->m_pure_pursuit_node_ptr);

    // 转换从rust来的消息
    autoware_auto_msgs::msg::Trajectory trajetory_msg = Convert(trajectory);
    autoware_auto_msgs::msg::VehicleKinematicState state = Convert(vehicle_kinematic_state);

    // 更新消息
    m_pure_pursuit_node_ptr->on_trajectory(std::make_shared<autoware_auto_msgs::msg::Trajectory>(trajetory_msg));
    m_pure_pursuit_node_ptr->on_state(std::make_shared<autoware_auto_msgs::msg::VehicleKinematicState>(state));
    m_pure_pursuit_node_ptr->retry_compute();
    return Convert(m_pure_pursuit_node_ptr->command);
}