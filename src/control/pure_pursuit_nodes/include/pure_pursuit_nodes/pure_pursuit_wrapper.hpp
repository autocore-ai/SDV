#pragma once
#include <string>
#include <memory>
#include <cstdint>
#include "pure_pursuit_nodes/pure_pursuit_node.hpp"
#include <message_convert/rust_msgs.hpp>

class PurePursuitNodeWrapper
{
public:
    // 创建PurePursuitNode
    PurePursuitNodeWrapper();
    // 更新车辆模型
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleControlCommand updateCommand(zenoh_flow::autoware_auto::autoware_auto_msgs_Trajectory trajectory, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState vehicle_kinematic_state);
private:
    std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> m_pure_pursuit_node_ptr;
};
