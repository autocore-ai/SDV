#pragma once
#include <string>
#include <memory>
#include <cstdint>
#include "lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp"
#include <message_convert/rust_msgs.hpp>

class Lanelet2GlobalPlannerNodeWrapper
{
public:
    // 创建Lanelet2GlobalPlannerNodeWrapper
    Lanelet2GlobalPlannerNodeWrapper();
    // 更新目标Pose
    zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute updateGoalPoseAndCurrentPose(zenoh_flow::autoware_auto::geometry_msgs_PoseStamped goal_pose_msg, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState current_pose_msg);
private:
    std::shared_ptr<autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode> m_lanelet2_planner_node_ptr;
};

LANELET2_GLOBAL_PLANNER_NODES_PUBLIC void* getLanelet2GlobalPlannerNodeWrapper();
