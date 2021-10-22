#pragma once
#include "lane_planner_nodes/lane_planner_node.hpp"
#include "lane_planner_nodes/visibility_control.hpp"
#include <message_convert/rust_msgs.hpp>
#include <cstdint>

class LanePlannerNodeWrapper
{
public:
    // 创建LanePlannerNodeWrapper
    LanePlannerNodeWrapper();
   private:
    std::shared_ptr<autoware::lane_planner_nodes::LanePlannerNode> m_lanelet2_planner_node_ptr;
    void spin();
};

LANE_PLANNER_NODES_PUBLIC void shutdown(int sig);
LANE_PLANNER_NODES_PUBLIC void* getLanePlannerNodeWrapper();