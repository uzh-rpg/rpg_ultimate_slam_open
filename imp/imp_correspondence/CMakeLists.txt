project(imp_correspondence)
cmake_minimum_required(VERSION 2.8.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/imp/correspondence/epipolar_matcher.hpp
  include/imp/correspondence/epipolar_sampling.hpp
  include/imp/correspondence/klt.hpp
  include/imp/correspondence/patch_score.hpp
  include/imp/correspondence/patch_utils.hpp
  include/imp/correspondence/sparse_stereo.hpp
  )

set(SOURCES
  src/epipolar_matcher.cpp
  src/epipolar_sampling.cpp
  src/klt.cpp
  src/sparse_stereo.cpp
  )

IF(DEFINED ENV{ARM_ARCHITECTURE})
  LIST(APPEND SOURCES src/klt_neon.cpp)
ENDIF()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_epipolar_sampling test/test_epipolar_sampling.cpp)
target_link_libraries(test_epipolar_sampling ${PROJECT_NAME})

catkin_add_gtest(test_epipolar_matcher test/test_epipolar_matcher.cpp)
target_link_libraries(test_epipolar_matcher ${PROJECT_NAME})

catkin_add_gtest(test_patch_utils test/test_patch_utils.cpp)
target_link_libraries(test_patch_utils ${PROJECT_NAME})

catkin_add_gtest(test_klt test/test_klt.cpp)
target_link_libraries(test_klt ${PROJECT_NAME})

catkin_add_gtest(test_sparse_stereo test/test_sparse_stereo.cpp)
target_link_libraries(test_sparse_stereo ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

