cmake_minimum_required(VERSION 3.5)
project(turtlesim_sink)

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

set(CMAKE_SCRIPTS_DIR ${CMAKE_SOURCE_DIR}/cmake)

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
  COMMAND ${CMAKE_COMMAND} -DHEADER=${cxxbridge_header_out} -P ${CMAKE_SCRIPTS_DIR}/Patcher.cmake
  COMMENT "Generating CXX bridge:"
)

set(cxxwrapper_lib ${CMAKE_SOURCE_DIR}/../target/release/libturtlesim_sink.a)

add_custom_target(WrapperLib ALL DEPENDS ${cxxwrapper_lib})
add_custom_command(
  OUTPUT ${cxxwrapper_lib}
  COMMAND cargo build --release
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  DEPENDS ${cxxbridge_in}
  COMMENT "Generating Rust library:"
  )

add_library(CxxWrapper STATIC IMPORTED GLOBAL)
add_dependencies(CxxWrapper WrapperLib)
set_target_properties(CxxWrapper
  PROPERTIES
  IMPORTED_LOCATION ${cxxwrapper_lib}
)

ament_auto_add_library(${PROJECT_NAME} SHARED src/sink.cpp src/wrapper.cpp)
add_dependencies(${PROJECT_NAME} CxxBridge)
target_link_libraries(${PROJECT_NAME} CxxWrapper)

ament_auto_package()
