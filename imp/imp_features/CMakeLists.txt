project(imp_features)
cmake_minimum_required(VERSION 2.8.0)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ze_setup)
include(ze_brisk)

#############
# LIBRARIES #
#############
set(HEADERS
  include/imp/features/fast_detector.hpp
  include/imp/features/fast_masked.hpp
  include/imp/features/feature_detector.hpp
  include/imp/features/feature_descriptor.hpp
  include/imp/features/keypoints_wrapper.hpp
  include/imp/features/occupancy_grid_2d.hpp
  include/imp/features/opencv_detector_utils.hpp
  )

set(SOURCES
  src/fast_detector.cpp
  src/feature_detector.cpp
  src/feature_descriptor.cpp
  src/occupancy_grid_2d.cpp
  src/opencv_detector_utils.cpp
  )

if(ZE_USE_BRISK)
  list(APPEND HEADERS 
    include/imp/features/brisk_detector.hpp
    include/imp/features/brisk_descriptor.hpp)
  list(APPEND SOURCES
    src/brisk_detector.cpp
    src/brisk_descriptor.cpp)
endif()

IF(DEFINED ENV{ARM_ARCHITECTURE})
  LIST(APPEND SOURCES src/fast_masked_neon.cpp)
ELSE()
  LIST(APPEND SOURCES src/fast_masked_sse.cpp)
ENDIF()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_fast_masked test/test_fast_masked.cpp)
target_link_libraries(test_fast_masked ${PROJECT_NAME})

catkin_add_gtest(test_feature_detector test/test_feature_detector.cpp)
target_link_libraries(test_feature_detector ${PROJECT_NAME})

catkin_add_gtest(test_feature_descriptor test/test_feature_descriptor.cpp)
target_link_libraries(test_feature_descriptor ${PROJECT_NAME})

catkin_add_gtest(test_occupancy_grid_2d test/test_occupancy_grid_2d.cpp)
target_link_libraries(test_occupancy_grid_2d ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

