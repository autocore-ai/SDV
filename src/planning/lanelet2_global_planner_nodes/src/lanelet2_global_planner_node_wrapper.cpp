#include "lanelet2_global_planner_nodes/lanelet2_global_planner_node_wrapper.hpp"
#include "lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp"
#include <message_convert/rust_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <message_convert/geometry_msgs.hpp>
#include <message_convert/autoware_auto_msgs.hpp>

Lanelet2GlobalPlannerNodeWrapper::Lanelet2GlobalPlannerNodeWrapper() {
    rclcpp::init(0, nullptr);
    //组装options
    rclcpp::NodeOptions options;
    m_lanelet2_planner_node_ptr = std::make_shared<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode>(options);
}

zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute Lanelet2GlobalPlannerNodeWrapper::updateGoalPoseAndCurrentPose(
    zenoh_flow::autoware_auto::geometry_msgs_PoseStamped goal_pose_msg, 
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState current_pose_msg) {
    
    rclcpp::spin_some(this->m_lanelet2_planner_node_ptr);

    geometry_msgs::msg::PoseStamped goal_pose_ros_msg = Convert(goal_pose_msg);
    autoware_auto_msgs::msg::VehicleKinematicState current_pose_ros_msg = Convert(current_pose_msg);

    m_lanelet2_planner_node_ptr->current_pose_cb(std::make_shared<autoware_auto_msgs::msg::VehicleKinematicState>(current_pose_ros_msg));
    m_lanelet2_planner_node_ptr->goal_pose_cb(std::make_shared<geometry_msgs::msg::PoseStamped>(goal_pose_ros_msg));
    
    return Convert(m_lanelet2_planner_node_ptr->m_global_path_msg);
}

void* getLanelet2GlobalPlannerNodeWrapper(){
    return new Lanelet2GlobalPlannerNodeWrapper();
}