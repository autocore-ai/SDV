#include "simple_planning_simulator/simple_planning_simulator_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time_source.hpp>
#include <array>
#include <memory>
#include <message_convert/geometry_msgs.hpp>
#include <message_convert/autoware_auto_msgs.hpp>

#include <cstdint>

SimplePlanningSimulatorWrapper::SimplePlanningSimulatorWrapper() {
    // TODO：将options中的json内容解析成配置参数
    // 现阶段先写死
    rclcpp::init(0, nullptr);
    
    //组装options
    rclcpp::NodeOptions options;
    // TODO: 参数需要换掉
    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

    paramters.push_back(rclcpp::Parameter("simulated_frame_id", "base_link"));
    paramters.push_back(rclcpp::Parameter("origin_frame_id", "odom"));
    paramters.push_back(rclcpp::Parameter("vehicle_model_type", "IDEAL_STEER_VEL"));
    paramters.push_back(rclcpp::Parameter("initialize_source", "INITIAL_POSE_TOPIC"));
    paramters.push_back(rclcpp::Parameter("timer_sampling_time_ms", 25));
    paramters.push_back(rclcpp::Parameter("add_measurement_noise", false));
    paramters.push_back(rclcpp::Parameter("vel_lim", 30.0));
    paramters.push_back(rclcpp::Parameter("vel_rate_lim", 30.0));
    paramters.push_back(rclcpp::Parameter("steer_lim", 0.6));
    paramters.push_back(rclcpp::Parameter("steer_rate_lim", 6.28));
    paramters.push_back(rclcpp::Parameter("acc_time_delay", 0.1));
    paramters.push_back(rclcpp::Parameter("acc_time_constant", 0.1));
    paramters.push_back(rclcpp::Parameter("steer_time_delay", 0.1));
    paramters.push_back(rclcpp::Parameter("steer_time_constant", 0.1));

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
    autoware_auto_msgs::msg::VehicleKinematicState vehicle_kinematic_state_msg;
    autoware_auto_msgs::msg::VehicleStateReport vehicle_state_report_msg;
    std::tie(vehicle_kinematic_state_msg, vehicle_state_report_msg) = m_simple_planning_simulator_ptr->update_vehicle_model();
    m_vehicle_kinematic_state_msg = Convert(vehicle_kinematic_state_msg);
    m_vehicle_state_report_msg = Convert(vehicle_state_report_msg);
}

autoware_auto_msgs__msg__VehicleKinematicState SimplePlanningSimulatorWrapper::getVehicleKinematicState() {
    return this->m_vehicle_kinematic_state_msg;
}
autoware_auto_msgs__msg__VehicleStateReport SimplePlanningSimulatorWrapper::getVehicleStateReport() {
    return this->m_vehicle_state_report_msg;
}

void* getSimplePlanningSimulatorWrapper(){
    return new SimplePlanningSimulatorWrapper();
}