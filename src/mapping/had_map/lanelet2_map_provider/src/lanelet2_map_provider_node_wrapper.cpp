#include "lanelet2_map_provider/lanelet2_map_provider_node_wrapper.hpp"
#include <csignal>

Lanelet2MapProviderNodeWrapper::Lanelet2MapProviderNodeWrapper() {
    rclcpp::init(0, nullptr);
    //组装options
    rclcpp::NodeOptions options;

    std::vector<rclcpp::Parameter> paramters = std::vector<rclcpp::Parameter>();

    // 由于写死，这里的值需要根据具体情况调整
    paramters.push_back(rclcpp::Parameter("map_osm_file", ""));
    paramters.push_back(rclcpp::Parameter("origin_offset_lat", 0.0));
    paramters.push_back(rclcpp::Parameter("origin_offset_lon", 0.0));
    paramters.push_back(rclcpp::Parameter("latitude", 37.380811523812845));
    paramters.push_back(rclcpp::Parameter("longitude", -121.90840595108715));
    paramters.push_back(rclcpp::Parameter("elevation", 16.0));

    options.parameter_overrides(paramters);

    m_lanelet2_map_provider_node_ptr = std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapProviderNode>(options);

    std::thread{std::bind(&Lanelet2MapProviderNodeWrapper::spin, this)}.detach();
    
    signal(SIGINT, shutdown);
}

void Lanelet2MapProviderNodeWrapper::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(m_lanelet2_map_provider_node_ptr);
  }
}

void shutdown(int sig)
{
  (void)sig;
  exit(0);
}

void* getLanelet2MapProviderNodeWrapper() {
    return new Lanelet2MapProviderNodeWrapper();
}