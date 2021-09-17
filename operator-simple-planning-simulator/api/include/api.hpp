#pragma once

#include <iostream>
#include <memory>
#include <simple_planning_simulator/simple_planning_simulator_core.hpp>

using SimplePlanningSimulator = simulation::simple_planning_simulator::SimplePlanningSimulator;

class Api
{
public:
    Api();
    ~Api();
    void SpinSome();

private:
    std::shared_ptr<SimplePlanningSimulator> api;
};

void *GetApi();
