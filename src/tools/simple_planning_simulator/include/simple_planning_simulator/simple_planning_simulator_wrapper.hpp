#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <autoware_auto_msgs/msg/vehicle_control_command.h>
#include <autoware_auto_msgs/msg/vehicle_state_command.h>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.h>
#include <autoware_auto_msgs/msg/vehicle_state_report.h>
#include "simple_planning_simulator/simple_planning_simulator_core.hpp"
#include "simple_planning_simulator/visibility_control.hpp"

#include <cstdint>

class SimplePlanningSimulatorWrapper
{
public:
    // 创建SimplePlanningSimulatorWrapper
    SimplePlanningSimulatorWrapper();
    // 当指定起始点，初始化车辆模型
    void initialPose(geometry_msgs__msg__PoseWithCovarianceStamped msg);
    // 更新车辆模型
    void updateVehicleModel(autoware_auto_msgs__msg__VehicleControlCommand vehicle_control_command, 
        autoware_auto_msgs__msg__VehicleStateCommand vehicle_state_command);
    autoware_auto_msgs__msg__VehicleKinematicState getVehicleKinematicState();
    autoware_auto_msgs__msg__VehicleStateReport getVehicleStateReport();

private:
    std::shared_ptr<simulation::simple_planning_simulator::SimplePlanningSimulator> m_simple_planning_simulator_ptr;
    autoware_auto_msgs__msg__VehicleKinematicState m_vehicle_kinematic_state_msg;
    autoware_auto_msgs__msg__VehicleStateReport m_vehicle_state_report_msg;
};

PLANNING_SIMULATOR_PUBLIC void* getSimplePlanningSimulatorWrapper();
