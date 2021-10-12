#pragma once
#include "simple_planning_simulator/simple_planning_simulator_core.hpp"
#include "simple_planning_simulator/visibility_control.hpp"
#include <message_convert/rust_msgs.hpp>
#include <cstdint>

class SimplePlanningSimulatorWrapper
{
public:
    // 创建SimplePlanningSimulatorWrapper
    SimplePlanningSimulatorWrapper();
    // 当指定起始点，初始化车辆模型
    void initialPose(zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovarianceStamped msg);
    // 更新车辆模型
    void updateVehicleModel(zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleControlCommand vehicle_control_command, 
        zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateCommand vehicle_state_command);
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState getVehicleKinematicState();
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport getVehicleStateReport();

private:
    std::shared_ptr<simulation::simple_planning_simulator::SimplePlanningSimulator> m_simple_planning_simulator_ptr;
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleKinematicState m_vehicle_kinematic_state_msg;
    zenoh_flow::autoware_auto::autoware_auto_msgs_VehicleStateReport m_vehicle_state_report_msg;
};

PLANNING_SIMULATOR_PUBLIC void* getSimplePlanningSimulatorWrapper();
