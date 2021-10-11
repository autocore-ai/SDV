cmake_minimum_required(VERSION 3.5)
project(turtlesim_sink)

find_program(CXXBRIDGE cxxbridge REQUIRED PATHS $ENV{HOME}/.cargo/bin)
message(STATUS "Using cxxbridge: ${CXXBRIDGE}")

set(cxxbridge_in ${CMAKE_SOURCE_DIR}/src/lib.rs)
set(cxxbridge_source_out ${CMAKE_SOURCE_DIR}/src/wrapper.cpp)
set(cxxbridge_header_out ${CMAKE_SOURCE_DIR}/include/wrapper.hpp)

add_custom_target(CxxBridge ALL DEPENDS ${cxxbridge_source_out} ${cxxbridge_header_out})
add_custom_command(
  OUTPUT ${cxxbridge_header_out} ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --output ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --header --output ${cxxbridge_header_out}
  COMMENT "Generating CXX bridge: ${cxxbridge_source_out} ${cxxbridge_header_out}"
  DEPENDS ${cxxbridge_in}
)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_native SHARED src/turtle_frame.cpp src/turtle.cpp)

ament_auto_package()
