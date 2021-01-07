cmake_minimum_required(VERSION 2.8.3)
project(ze_imu)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

# This macro ensures modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/imu/imu_model.hpp
  include/ze/imu/imu_rig.hpp
  include/ze/imu/imu_intrinsic_model.hpp
  include/ze/imu/imu_noise_model.hpp
  include/ze/imu/accelerometer_model.hpp
  include/ze/imu/gyroscope_model.hpp
  include/ze/imu/imu_yaml_serialization.hpp
  include/ze/imu/imu_buffer.hpp
  include/ze/imu/imu_types.hpp
  )

set(SOURCES
    src/imu_model.cpp
    src/imu_rig.cpp
    src/imu_intrinsic_model.cpp
    src/imu_noise_model.cpp
    src/accelerometer_model.cpp
    src/gyroscope_model.cpp
    src/imu_yaml_serialization.cpp
    src/imu_buffer.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########

catkin_add_gtest(test_accelerometer_model test/test_accelerometer_model.cpp)
target_link_libraries(test_accelerometer_model ${PROJECT_NAME})

catkin_add_gtest(test_gyroscope_model test/test_gyroscope_model.cpp)
target_link_libraries(test_gyroscope_model ${PROJECT_NAME})

catkin_add_gtest(test_imu_model test/test_imu_model.cpp)
target_link_libraries(test_imu_model ${PROJECT_NAME})

catkin_add_gtest(test_yaml_serialization test/test_yaml_serialization.cpp)
target_link_libraries(test_yaml_serialization ${PROJECT_NAME})

catkin_add_gtest(test_intrinsic_model test/test_intrinsic_model.cpp)
target_link_libraries(test_intrinsic_model ${PROJECT_NAME})

catkin_add_gtest(test_noise_model test/test_noise_model.cpp)
target_link_libraries(test_noise_model ${PROJECT_NAME})

catkin_add_gtest(test_imu_buffer test/test_imu_buffer.cpp)
target_link_libraries(test_imu_buffer ${PROJECT_NAME})

catkin_add_gtest(test_imu_types test/test_imu_types.cpp)
target_link_libraries(test_imu_types ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
