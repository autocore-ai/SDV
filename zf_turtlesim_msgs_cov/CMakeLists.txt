cmake_minimum_required(VERSION 3.5)
project(zf_turtlesim_msgs_cov)

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

ament_auto_find_build_dependencies()


find_program(CXXBRIDGE cxxbridge REQUIRED PATHS $ENV{HOME}/.cargo/bin)

message(STATUS "Using cxxbridge: ${CXXBRIDGE}")

set(wrapper_dir  ${CMAKE_SOURCE_DIR})
set(include_dir  ${wrapper_dir}/include)

set(cxxbridge_in         ${wrapper_dir}/src/lib.rs)
set(cxxbridge_source_out ${CMAKE_SOURCE_DIR}/src/wrapper.cpp)
set(cxxbridge_header_out ${CMAKE_SOURCE_DIR}/include/wrapper.hpp)

add_custom_target(CxxBridge ALL
  DEPENDS ${cxxbridge_header_out} ${cxxbridge_source_out})

add_custom_command(
  OUTPUT ${cxxbridge_header_out} ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --output ${cxxbridge_source_out}
  COMMAND ${CXXBRIDGE} ${cxxbridge_in} --header --output ${cxxbridge_header_out}
  COMMENT "Generating CXX bridge:"
)

ament_auto_add_library(${PROJECT_NAME} SHARED src/cov.cpp)

add_dependencies(${PROJECT_NAME} CxxBridge)

ament_auto_package()
