project(ze_ceres_backend)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

# OpenMP
find_package(OpenMP QUIET)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found")
  add_definitions(-DUSE_OPENMP)
endif ()

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/nlls/estimator.hpp
  include/ze/nlls/estimator_impl.hpp
  include/ze/nlls/estimator_types.hpp
  include/ze/nlls/ceres_iteration_callback.hpp
  include/ze/nlls/error_interface.hpp
  include/ze/nlls/homogeneous_point_error.hpp
  include/ze/nlls/homogeneous_point_local_parameterization.hpp
  include/ze/nlls/homogeneous_point_parameter_block.hpp
  include/ze/nlls/imu_error.hpp
  include/ze/nlls/local_parameterization_additional_interfaces.hpp
  include/ze/nlls/map.hpp
  include/ze/nlls/marginalization_error.hpp
  include/ze/nlls/marginalization_error_impl.hpp
  include/ze/nlls/parameter_block.hpp
  include/ze/nlls/pose_error.hpp
  include/ze/nlls/pose_local_parameterization.hpp
  include/ze/nlls/pose_parameter_block.hpp
  include/ze/nlls/relative_pose_error.hpp
  include/ze/nlls/reprojection_error.hpp
  include/ze/nlls/reprojection_error_impl.hpp
  include/ze/nlls/reprojection_error_base.hpp
  include/ze/nlls/speed_and_bias_error.hpp
  include/ze/nlls/speed_and_bias_parameter_block.hpp
  )

set(SOURCES
  src/estimator.cpp
  src/homogeneous_point_error.cpp
  src/homogeneous_point_parameter_block.cpp
  src/homogeneous_point_local_parameterization.cpp
  src/imu_error.cpp
  src/local_parameterization_additional_interfaces.cpp
  src/map.cpp
  src/marginalization_error.cpp
  src/pose_error.cpp
  src/pose_parameter_block.cpp
  src/pose_local_parameterization.cpp
  src/relative_pose_error.cpp
  src/speed_and_bias_error.cpp
  src/speed_and_bias_parameter_block.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# EXPORT #
##########
cs_install()
cs_export()
