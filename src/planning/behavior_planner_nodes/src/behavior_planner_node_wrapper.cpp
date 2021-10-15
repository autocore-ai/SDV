#include "behavior_planner_nodes/behavior_planner_node_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time_source.hpp>
#include <array>
#include <memory>
#include <message_convert/geometry_msgs.hpp>
#include <message_convert/autoware_auto_msgs.hpp>
#include <message_convert/rust_msgs.hpp>
#include <cstdint>
#include <algorithm>

BehaviorPlannerNodeWrapper::BehaviorPlannerNodeWrapper() {
    // TODO：将options中的json内容解析成配置参数
    // 现阶段先写死
    rclcpp::init(0, nullptr);
    //组装options
    rclcpp::NodeOptions options;

    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

    paramters.push_back(rclcpp::Parameter("enable_object_collision_estimator", true));
    paramters.push_back(rclcpp::Parameter("heading_weight", 0.1));
    paramters.push_back(rclcpp::Parameter("goal_distance_thresh", 3.0));
    paramters.push_back(rclcpp::Parameter("stop_velocity_thresh", 2.0));
    paramters.push_back(rclcpp::Parameter("subroute_goal_offset_lane2parking", 7.6669));
    paramters.push_back(rclcpp::Parameter("subroute_goal_offset_parking2lane", 7.6669));
    paramters.push_back(rclcpp::Parameter("vehicle.cg_to_front_m", 1.228));
    paramters.push_back(rclcpp::Parameter("vehicle.cg_to_rear_m", 1.5618));
    paramters.push_back(rclcpp::Parameter("vehicle.front_overhang_m", 0.5));
    paramters.push_back(rclcpp::Parameter("vehicle.rear_overhang_m", 0.5));

    options.parameter_overrides(paramters);

    m_behavior_planner_node_ptr = std::make_shared<autoware::behavior_planner_nodes::BehaviorPlannerNode>(options);
}
// 更新Message
void BehaviorPlannerNodeWrapper::UpdateBehaviorPlanner(
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState state_msg, 
    zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute route_msg, 
    zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory lane_trajectory_msg, 
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport  vehicle_state_report_msg) 
{
    if (!(state_msg.header.stamp.nanosec == 0 && state_msg.header.stamp.sec == 0))
    {
        autoware_auto_msgs::msg::VehicleKinematicState vehicle_kinematic_state_msg = Convert(state_msg);
        m_behavior_planner_node_ptr ->on_ego_state(std::make_shared<autoware_auto_msgs::msg::VehicleKinematicState>(vehicle_kinematic_state_msg));
    }

    if (!(route_msg.header.stamp.nanosec == 0 && route_msg.header.stamp.sec == 0))
    {
        autoware_auto_msgs::msg::HADMapRoute had_map_route_msg = Convert(route_msg);
        m_behavior_planner_node_ptr ->on_route(std::make_shared<autoware_auto_msgs::msg::HADMapRoute>(had_map_route_msg));
    }

    if (!(lane_trajectory_msg.header.stamp.nanosec == 0 && lane_trajectory_msg.header.stamp.sec == 0))
    {
        autoware_auto_msgs::msg::Trajectory trajectory_msg = Convert(lane_trajectory_msg);
        m_behavior_planner_node_ptr ->on_lane_trajectory(std::make_shared<autoware_auto_msgs::msg::Trajectory>(trajectory_msg));
    }

    if (!(vehicle_state_report_msg.stamp.nanosec == 0 && vehicle_state_report_msg.stamp.sec == 0))
    {
        autoware_auto_msgs::msg::VehicleStateReport report_msg = Convert(vehicle_state_report_msg);
        m_behavior_planner_node_ptr ->on_vehicle_state_report(std::make_shared<autoware_auto_msgs::msg::VehicleStateReport>(report_msg));
    }
}



zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory BehaviorPlannerNodeWrapper::getTrajectoryMsg() {
    return Convert(m_behavior_planner_node_ptr->m_trajectory_msg);
}
zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory BehaviorPlannerNodeWrapper::getDebugTrajectoryMsg() {
    return Convert(m_behavior_planner_node_ptr->m_debug_trajectory_msg);
}
zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory BehaviorPlannerNodeWrapper::getDebugCheckpointsMsg() {
    return Convert(m_behavior_planner_node_ptr->m_debug_checkpoints_msg);
}
zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute BehaviorPlannerNodeWrapper::getDebugSubrouteMsg() {
    return Convert(m_behavior_planner_node_ptr->m_debug_subroute_msg);
}
zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand BehaviorPlannerNodeWrapper::getVehicleStateCommandMsg() {
    return Convert(m_behavior_planner_node_ptr->m_vehicle_state_command_msg);
}

void* getBehaviorPlannerNodeWrapper(){
    return new BehaviorPlannerNodeWrapper();
}
