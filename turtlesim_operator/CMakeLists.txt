cmake_minimum_required(VERSION 3.5)
project(turtlesim_operator)

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
find_package(Qt5 REQUIRED COMPONENTS Gui)

include_directories(include ${Qt5Gui_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/turtle.cpp)

ament_auto_package()
