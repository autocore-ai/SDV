#include <api.hpp>
#include <rclcpp/rclcpp.hpp>

static std::shared_ptr<Api> sp;

Api::Api()
{
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
    node_options.append_parameter_override("cg_to_rear_m", 1.5f);
    node_options.append_parameter_override("vehicle_model_type", "IDEAL_STEER_VEL");
    api = std::make_shared<SimplePlanningSimulator>(node_options);
}

Api::~Api()
{
    rclcpp::shutdown();
}

void Api::SpinSome()
{
    rclcpp::spin_some(api->shared_from_this());
}

void *GetApi()
{
    sp = std::make_shared<Api>();
    return sp.get();
}
