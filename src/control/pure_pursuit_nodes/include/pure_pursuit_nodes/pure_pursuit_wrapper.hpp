#pragma once
#include <string>
#include <memory>
#include <cstdint>
#include "pure_pursuit_nodes/pure_pursuit_node.hpp"
#include <autoware_auto_msgs/msg/vehicle_control_command.h>

#include <autoware_auto_msgs/msg/trajectory.h>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.h>
class PurePursuitNodeWrapper
{
public:
    // 创建PurePursuitNode
    PurePursuitNodeWrapper();
    // 更新车辆模型
    autoware_auto_msgs__msg__VehicleControlCommand updateCommand(autoware_auto_msgs__msg__Trajectory trajectory, 
        autoware_auto_msgs__msg__VehicleKinematicState vehicle_kinematic_state);
private:
    std::shared_ptr<autoware::motion::control::pure_pursuit_nodes::PurePursuitNode> m_pure_pursuit_node_ptr;
};
