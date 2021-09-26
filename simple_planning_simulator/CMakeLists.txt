cmake_minimum_required(VERSION 3.5)
project(simple_planning_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(message_convert REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include "${CMAKE_CURRENT_SOURCE_DIR}/../message_convert/include/")
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/simple_planning_simulator_wrapper.cpp
    src/simple_planning_simulator_core.cpp
    src/vehicle_model/sim_model_interface.cpp
    src/vehicle_model/sim_model_ideal_steer_vel.cpp
    src/vehicle_model/sim_model_ideal_steer_acc.cpp
    src/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
    src/vehicle_model/sim_model_delay_steer_acc.cpp
    src/vehicle_model/sim_model_delay_steer_acc_geared.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp message_convert tf2_geometry_msgs)

# target_link_libraries(${PROJECT_NAME} PUBLIC message_convert)

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

ament_package()
