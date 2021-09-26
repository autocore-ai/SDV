cmake_minimum_required(VERSION 3.6)

project(simple_planning_simulator)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(message_convert REQUIRED)
ament_auto_find_build_dependencies()


# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/simple_planning_simulator/simple_planning_simulator_core.hpp
  include/simple_planning_simulator/simple_planning_simulator_wrapper.hpp
  include/simple_planning_simulator/visibility_control.hpp
  src/simple_planning_simulator/simple_planning_simulator_core.cpp
  src/simple_planning_simulator/simple_planning_simulator_wrapper.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_interface.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} message_convert)
autoware_set_compile_options(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast) # RCLCPP_ERROR_THROTTLE() has built-in old-style casts.


# Node executable
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "simulation::simple_planning_simulator::SimplePlanningSimulator"
#   EXECUTABLE ${PROJECT_NAME}_exe
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
