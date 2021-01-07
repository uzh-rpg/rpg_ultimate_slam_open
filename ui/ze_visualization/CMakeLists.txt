cmake_minimum_required(VERSION 2.8.3)
project(ze_visualization)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/visualization/viz_common.hpp
  include/ze/visualization/viz_interface.hpp
  include/ze/visualization/viz_ros.hpp
  include/ze/visualization/viz_ros_utils.hpp
  )

set(SOURCES
  src/viz_common.cpp 
  src/viz_ros.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############
cs_add_executable(viz_example_node src/viz_example_node.cpp)
target_link_libraries(viz_example_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
