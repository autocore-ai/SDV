cmake_minimum_required(VERSION 3.5)
project(turtlesim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

ament_auto_find_build_dependencies()

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RotateAbsolute.action"
  "msg/Color.msg"
  "msg/Pose.msg"
  "srv/Kill.srv"
  "srv/SetPen.srv"
  "srv/Spawn.srv"
  "srv/TeleportAbsolute.srv"
  "srv/TeleportRelative.srv")

set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "rclcpp_action" "std_msgs" "std_srvs")

set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
  include/turtlesim/turtle_frame.h
)

# qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


# add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
# target_link_libraries(turtlesim_node Qt5::Widgets)
# ament_target_dependencies(turtlesim_node ${dependencies})
# rosidl_target_interfaces(turtlesim_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_executable(turtlesim_source_native_exe src/turtlesim_source.cpp)
ament_auto_add_library(turtlesim_source_native SHARED src/turtlesim_source.cpp)
rosidl_target_interfaces(turtlesim_source_native_exe ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(turtlesim_source_native ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_executable(turtlesim_operator_native_exe src/turtlesim_operator.cpp)
ament_auto_add_library(turtlesim_operator_native SHARED src/turtlesim_operator.cpp)

ament_auto_add_executable(turtlesim_sink_native_exe src/turtlesim_sink.cpp)
ament_auto_add_library(turtlesim_sink_native SHARED src/turtlesim_sink.cpp)
target_link_libraries(turtlesim_sink_native_exe Qt5::Widgets)
target_link_libraries(turtlesim_sink_native Qt5::Widgets)

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
