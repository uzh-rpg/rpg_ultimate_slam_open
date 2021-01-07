project(imp_feature_matching)
cmake_minimum_required(VERSION 2.8.0)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/imp/feature_matching/brute_force_matcher.hpp
  include/imp/feature_matching/hamming.hpp
  include/imp/feature_matching/hamming-inl.hpp
  include/imp/feature_matching/matching_types.hpp
  include/imp/feature_matching/matching_viz.hpp
  )

set(SOURCES
  src/brute_force_matcher.cpp
  src/matching_types.cpp
  src/matching_viz.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_brute_force_matcher test/test_brute_force_matcher.cpp)
target_link_libraries(test_brute_force_matcher ${PROJECT_NAME})

catkin_add_gtest(test_hamming test/test_hamming.cpp)
target_link_libraries(test_hamming ${PROJECT_NAME})

catkin_add_gtest(test_matching_viz test/test_matching_viz.cpp)
target_link_libraries(test_matching_viz ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

