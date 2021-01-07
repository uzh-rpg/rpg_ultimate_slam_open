project(ze_ros)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/ros/rosbag_image_query.hpp
  include/ze/ros/pose_msg_bridge.hpp
  include/ze/ros/tf_bridge.hpp
  )

set(SOURCES
  src/rosbag_image_query.cpp
  src/pose_msg_bridge.cpp
  src/tf_bridge.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_rosbag_image_query test/test_rosbag_image_query.cpp)
target_link_libraries(test_rosbag_image_query ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
