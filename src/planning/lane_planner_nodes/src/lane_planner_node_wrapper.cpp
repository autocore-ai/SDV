#include "lane_planner_nodes/lane_planner_node_wrapper.hpp"

LanePlannerNodeWrapper::LanePlannerNodeWrapper(){
     rclcpp::init(0, nullptr);
    // 组装options
    rclcpp::NodeOptions options;

    // 组装参数
    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

    paramters.push_back(rclcpp::Parameter("heading_weight", 0.1));
    paramters.push_back(rclcpp::Parameter("lane_planner.trajectory_resolution", 2.0));
    paramters.push_back(rclcpp::Parameter("vehicle.cg_to_front_m", 1.0));
    paramters.push_back(rclcpp::Parameter("vehicle.cg_to_rear_m", 1.0));
    paramters.push_back(rclcpp::Parameter("vehicle.front_corner_stiffness", 0.1));
    paramters.push_back(rclcpp::Parameter("vehicle.rear_corner_stiffness", 0.1));
    paramters.push_back(rclcpp::Parameter("vehicle.mass_kg", 1500.0));
    paramters.push_back(rclcpp::Parameter("vehicle.yaw_inertia_kgm2", 12.0));
    paramters.push_back(rclcpp::Parameter("vehicle.width_m", 2.0));
    paramters.push_back(rclcpp::Parameter("vehicle.front_overhang_m", 0.5));
    paramters.push_back(rclcpp::Parameter("vehicle.rear_overhang_m", 0.5));
    paramters.push_back(rclcpp::Parameter("gaussian_smoother.standard_deviation", 1.0));
    paramters.push_back(rclcpp::Parameter("gaussian_smoother.kernel_size", 5));

    options.parameter_overrides(paramters);

    m_lanelet2_planner_node_ptr = std::make_shared<autoware::lane_planner_nodes::LanePlannerNode>(options);

    std::thread{std::bind(&LanePlannerNodeWrapper::spin, this)}.detach();
    
    signal(SIGINT, shutdown);
}

void LanePlannerNodeWrapper::spin() {
    while (rclcpp::ok())
    {
        rclcpp::spin_some(m_lanelet2_planner_node_ptr);
    }
}

void shutdown(int sig)
{
  (void)sig;
  exit(0);
}

void* getLanePlannerNodeWrapper(){
    return new LanePlannerNodeWrapper();
}