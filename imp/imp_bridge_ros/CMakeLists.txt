project(imp_bridge_ros)
cmake_minimum_required(VERSION 2.8.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

set(HEADERS
  include/imp/bridge/ros/cu_ros_bridge.hpp
  include/imp/bridge/ros/ros_bridge.hpp
  )

set(SOURCES
  src/ros_bridge.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME})

cs_install()
cs_export()
