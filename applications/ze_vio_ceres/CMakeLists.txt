project(ze_vio_ceres)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/vio_ceres/vio_ceres_backend_interface.hpp
  )

set(SOURCES
  src/vio_ceres_backend_interface.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###############
# EXECUTABLES #
###############

cs_add_executable(ze_vio_ceres_node src/vio_ceres_node.cpp)
target_link_libraries(ze_vio_ceres_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

##########
# GTESTS #
##########

##########
# EXPORT #
##########
cs_install()
cs_export()
