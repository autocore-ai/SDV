#pragma once
#include <string>
#include <memory>
#include <cstdint>
#include "lanelet2_map_provider/lanelet2_map_provider_node.hpp"
#include <message_convert/rust_msgs.hpp>

class Lanelet2MapProviderNodeWrapper
{
public:
    // 创建Lanelet2MapProviderNodeWrapper
    Lanelet2MapProviderNodeWrapper();
private:
    std::shared_ptr<autoware::lanelet2_map_provider::Lanelet2MapProviderNode> m_lanelet2_map_provider_node_ptr;
    void spin();

};
LANELET2_MAP_PROVIDER_PUBLIC void shutdown(int sig);
LANELET2_MAP_PROVIDER_PUBLIC void* getLanelet2MapProviderNodeWrapper();
