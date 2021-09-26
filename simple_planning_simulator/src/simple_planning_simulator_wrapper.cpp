#include "simple_planning_simulator/simple_planning_simulator_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <array>
#include <memory>
#include "message_convert/geometry_msgs.hpp"
#include "message_convert/autoware_auto_msgs.hpp"

SimplePlanningSimulatorWrapper::SimplePlanningSimulatorWrapper(std::string options_str) {
    // TODO：将options中的json内容解析成配置参数
    // 现阶段先写死

    rclcpp::init(0, nullptr);
    
    //组装options
    rclcpp::NodeOptions options;

    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();
    paramters.push_back(rclcpp::Parameter("map_osm_file", 1));
    paramters.push_back(rclcpp::Parameter("origin_offset_lat", 1));
    paramters.push_back(rclcpp::Parameter("origin_offset_lon", 1));
    paramters.push_back(rclcpp::Parameter("latitude", 1));
    paramters.push_back(rclcpp::Parameter("longitude", 1));
    paramters.push_back(rclcpp::Parameter("elevation", 1));
    options.parameter_overrides(paramters);
    m_simple_planning_simulator_ptr = std::make_shared<simulation::simple_planning_simulator::SimplePlanningSimulator>(options);
}
// 当rviz指定起始点，初始化车辆模型
void SimplePlanningSimulatorWrapper::initialPose(geometry_msgs__msg__PoseWithCovarianceStamped msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped initinal_pose_msg = Convert(msg);
    m_simple_planning_simulator_ptr->on_initialpose(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(initinal_pose_msg));
}
// 更新车辆模型
void SimplePlanningSimulatorWrapper::updateVehicleModel(autoware_auto_msgs__msg__VehicleControlCommand vehicle_control_command, 
        autoware_auto_msgs__msg__VehicleStateCommand vehicle_state_command) {
    // 获得最新的消息
    rclcpp::spin_some(this->m_simple_planning_simulator_ptr);

    // 转换从rust来的消息
    autoware_auto_msgs::msg::VehicleControlCommand control_command = Convert(vehicle_control_command);
    autoware_auto_msgs::msg::VehicleStateCommand state_command = Convert(vehicle_state_command);

    // 更新车辆模型
    m_simple_planning_simulator_ptr->on_state_cmd(std::make_shared<autoware_auto_msgs::msg::VehicleStateCommand>(state_command));
    m_simple_planning_simulator_ptr->on_vehicle_cmd(std::make_shared<autoware_auto_msgs::msg::VehicleControlCommand>(control_command));
    m_simple_planning_simulator_ptr->update_vehicle_model();
}

RCLCPP_COMPONENTS_REGISTER_NODE(simulation::simple_planning_simulator::SimplePlanningSimulator)