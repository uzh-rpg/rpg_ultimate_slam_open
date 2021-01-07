cmake_minimum_required(VERSION 2.8.3)
project(ze_geometry)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

# quick fix to gcc bug / boost defining literals
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  #  gcc < 4.9 only
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")      
  endif()
endif()


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/geometry/align_points.hpp
  include/ze/geometry/align_poses.hpp
  include/ze/geometry/clam.hpp
  include/ze/geometry/epipolar_geometry.hpp
  include/ze/geometry/line.hpp
  include/ze/geometry/lsq_solver.hpp
  include/ze/geometry/lsq_solver-inl.hpp
  include/ze/geometry/lsq_state.hpp
  include/ze/geometry/pose_optimizer.hpp
  include/ze/geometry/pose_prior.hpp
  include/ze/geometry/ransac_relative_pose.hpp
  include/ze/geometry/robust_cost.hpp
  include/ze/geometry/triangulation.hpp
  )

set(SOURCES
  src/align_points.cpp
  src/align_poses.cpp
  src/clam.cpp
  src/line.cpp
  src/pose_optimizer.cpp
  src/ransac_relative_pose.cpp
  src/triangulation.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_align_points test/test_align_points.cpp)
target_link_libraries(test_align_points ${PROJECT_NAME})

catkin_add_gtest(test_align_poses test/test_align_poses.cpp)
target_link_libraries(test_align_poses ${PROJECT_NAME})

catkin_add_gtest(test_clam test/test_clam.cpp)
target_link_libraries(test_clam ${PROJECT_NAME})

catkin_add_gtest(test_line test/test_line.cpp)
target_link_libraries(test_line ${PROJECT_NAME})

catkin_add_gtest(test_lsq_state test/test_lsq_state.cpp)
target_link_libraries(test_lsq_state ${PROJECT_NAME})

catkin_add_gtest(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer ${PROJECT_NAME})

catkin_add_gtest(test_ransac_relative_pose test/test_ransac_relative_pose.cpp)
target_link_libraries(test_ransac_relative_pose ${PROJECT_NAME})

catkin_add_gtest(test_robust_cost test/test_robust_cost.cpp)
target_link_libraries(test_robust_cost ${PROJECT_NAME})

catkin_add_gtest(test_triangulation test/test_triangulation.cpp)
target_link_libraries(test_triangulation ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
