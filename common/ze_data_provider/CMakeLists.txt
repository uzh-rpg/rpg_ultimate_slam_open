project(ze_data_provider)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/data_provider/data_provider_base.hpp
  include/ze/data_provider/data_provider_factory.hpp
  include/ze/data_provider/data_provider_csv.hpp
  include/ze/data_provider/data_provider_rosbag.hpp
  include/ze/data_provider/data_provider_rostopic.hpp
  include/ze/data_provider/camera_imu_synchronizer_base.hpp
  include/ze/data_provider/camera_imu_synchronizer.hpp
  )

set(SOURCES
  src/data_provider_base.cpp
  src/data_provider_factory.cpp
  src/data_provider_csv.cpp
  src/data_provider_rosbag.cpp
  src/data_provider_rostopic.cpp
  src/camera_imu_synchronizer.cpp
  src/camera_imu_synchronizer_base.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_data_provider test/test_data_provider.cpp)
target_link_libraries(test_data_provider ${PROJECT_NAME})

catkin_add_gtest(test_camera_imu_synchronizer test/test_camera_imu_synchronizer.cpp)
target_link_libraries(test_camera_imu_synchronizer ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
