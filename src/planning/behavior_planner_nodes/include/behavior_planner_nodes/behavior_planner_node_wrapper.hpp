#include <message_convert/rust_msgs.hpp>
#include <behavior_planner_nodes/behavior_planner_node.hpp>
#include <memory>
class BehaviorPlannerNodeWrapper
{
public:
    // 创建BehaviorPlannerNode
    BehaviorPlannerNodeWrapper();

    // 更新消息
    void UpdateBehaviorPlanner(
        zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState state_msg, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute route_msg, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory lane_trajectory_msg, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport  vehicle_state_report_msg);
    
    zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory getTrajectoryMsg();
    zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory getDebugTrajectoryMsg();
    zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory getDebugCheckpointsMsg();
    zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapRoute getDebugSubrouteMsg();
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand getVehicleStateCommandMsg();

private:
    std::shared_ptr<autoware::behavior_planner_nodes::BehaviorPlannerNode> m_behavior_planner_node_ptr;
};

BEHAVIOR_PLANNER_NODES_PUBLIC void* getBehaviorPlannerNodeWrapper();