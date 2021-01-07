project(ze_vio_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

find_package(OpenCV REQUIRED)
list(APPEND catkin_LIBRARIES ${OpenCV_LIBRARIES})

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/vio_common/frame.hpp
  include/ze/vio_common/imu_integrator.hpp
  include/ze/vio_common/landmark_handle.hpp
  include/ze/vio_common/landmark_table.hpp
  include/ze/vio_common/landmark_triangulation.hpp
  include/ze/vio_common/landmark_types.hpp
  include/ze/vio_common/landmark_utils.hpp
  include/ze/vio_common/motion_type.hpp
  include/ze/vio_common/nframe.hpp
  include/ze/vio_common/nframe_handle.hpp
  include/ze/vio_common/nframe_table.hpp
  include/ze/vio_common/nframe_utils.hpp
  include/ze/vio_common/vio_visualizer.hpp
  include/ze/vio_common/localization.hpp
  )

set(SOURCES
  src/frame.cpp
  src/nframe.cpp
  src/nframe_table.cpp
  src/nframe_utils.cpp
  src/imu_integrator.cpp
  src/landmark_table.cpp
  src/landmark_triangulation.cpp
  src/landmark_types.cpp
  src/landmark_utils.cpp
  src/vio_visualizer.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_landmark_table test/test_landmark_table.cpp)
target_link_libraries(test_landmark_table ${PROJECT_NAME})

catkin_add_gtest(test_nframe_table test/test_nframe_table.cpp)
target_link_libraries(test_nframe_table ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
