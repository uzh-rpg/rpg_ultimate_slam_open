project(ze_common)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)


############
# SETTINGS #
############
include(ze_options)
message(STATUS "ZE_USE_OPENCV: ${ZE_USE_OPENCV}")
message(STATUS "ZE_USE_BRISK: ${ZE_USE_BRISK}")
message(STATUS "ZE_USE_GTSAM: ${ZE_USE_GTSAM}")
message(STATUS "ZE_SINGLE_PRECISION_FLOAT: ${ZE_SINGLE_PRECISION_FLOAT}")
message(STATUS "ZE_USE_ARRAYFIRE: ${ZE_USE_ARRAYFIRE}")
message(STATUS "ZE_DETERMINISTIC: ${ZE_DETERMINISTIC}")
message(STATUS "ZE_VIO_LIMITED: ${ZE_VIO_LIMITED}")

configure_file(include/ze/common/config.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ze/common/config.hpp)


#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/common/benchmark.hpp
  include/ze/common/buffer.hpp
  include/ze/common/buffer-inl.hpp
  include/ze/common/config.hpp
  include/ze/common/combinatorics.hpp
  include/ze/common/csv_trajectory.hpp
  include/ze/common/egg_timer.hpp
  include/ze/common/file_utils.hpp
  include/ze/common/logging.hpp
  include/ze/common/macros.hpp
  include/ze/common/manifold.hpp
  include/ze/common/math.hpp
  include/ze/common/matrix.hpp
  include/ze/common/nonassignable.hpp
  include/ze/common/noncopyable.hpp
  include/ze/common/numerical_derivative.hpp
  include/ze/common/path_utils.hpp
  include/ze/common/random.hpp
  include/ze/common/random_matrix.hpp
  include/ze/common/ringbuffer.hpp
  include/ze/common/ringbuffer-inl.hpp
  include/ze/common/ring_view.hpp
  include/ze/common/running_statistics.hpp
  include/ze/common/running_statistics_collection.hpp
  include/ze/common/signal_handler.hpp
  include/ze/common/statistics.hpp
  include/ze/common/stl_utils.hpp
  include/ze/common/string_utils.hpp
  include/ze/common/test_entrypoint.hpp
  include/ze/common/test_utils.hpp
  include/ze/common/test_thread_blocking.hpp
  include/ze/common/thread_pool.hpp
  include/ze/common/thread_safe_fifo.hpp
  include/ze/common/time_conversions.hpp
  include/ze/common/timer.hpp
  include/ze/common/timer_collection.hpp
  include/ze/common/timer_statistics.hpp
  include/ze/common/transformation.hpp
  include/ze/common/types.hpp
  include/ze/common/versioned_slot_handle.hpp
  include/ze/common/yaml_serialization.hpp
  )

set(SOURCES
  src/csv_trajectory.cpp
  src/matrix.cpp
  src/random.cpp
  src/signal_handler.cpp
  src/test_utils.cpp
  src/test_thread_blocking.cpp
  src/thread_pool.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


##########
# GTESTS #
##########
catkin_add_gtest(test_benchmark test/test_benchmark.cpp)
target_link_libraries(test_benchmark ${PROJECT_NAME})

catkin_add_gtest(test_buffer test/test_buffer.cpp)
target_link_libraries(test_buffer ${PROJECT_NAME})

catkin_add_gtest(test_csv_trajectory test/test_csv_trajectory.cpp)
target_link_libraries(test_csv_trajectory ${PROJECT_NAME})

catkin_add_gtest(test_manifold test/test_manifold.cpp)
target_link_libraries(test_manifold ${PROJECT_NAME})

catkin_add_gtest(test_matrix test/test_matrix.cpp)
target_link_libraries(test_matrix ${PROJECT_NAME})

catkin_add_gtest(test_numerical_derivative test/test_numerical_derivative.cpp)
target_link_libraries(test_numerical_derivative ${PROJECT_NAME})

catkin_add_gtest(test_random test/test_random.cpp)
target_link_libraries(test_random ${PROJECT_NAME})

catkin_add_gtest(test_random_matrix test/test_random_matrix.cpp)
target_link_libraries(test_random_matrix ${PROJECT_NAME})

catkin_add_gtest(test_running_statistics test/test_running_statistics.cpp)
target_link_libraries(test_running_statistics ${PROJECT_NAME})

catkin_add_gtest(test_statistics test/test_statistics.cpp)
target_link_libraries(test_statistics ${PROJECT_NAME})

catkin_add_gtest(test_stl_utils test/test_stl_utils.cpp)
target_link_libraries(test_stl_utils ${PROJECT_NAME})

catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
target_link_libraries(test_string_utils ${PROJECT_NAME})

catkin_add_gtest(test_test_utils test/test_test_utils.cpp)
target_link_libraries(test_test_utils ${PROJECT_NAME})

catkin_add_gtest(test_timer test/test_timer.cpp)
target_link_libraries(test_timer ${PROJECT_NAME})

catkin_add_gtest(test_transformation test/test_transformation.cpp)
target_link_libraries(test_transformation ${PROJECT_NAME})

catkin_add_gtest(test_thread_pool test/test_thread_pool.cpp)
target_link_libraries(test_thread_pool ${PROJECT_NAME})

catkin_add_gtest(test_thread_safe_fifo test/test_thread_safe_fifo.cpp)
target_link_libraries(test_thread_safe_fifo ${PROJECT_NAME})

catkin_add_gtest(test_versioned_slot_handle test/test_versioned_slot_handle.cpp)
target_link_libraries(test_versioned_slot_handle ${PROJECT_NAME})

catkin_add_gtest(test_ring_view test/test_ring_view.cpp)
target_link_libraries(test_ring_view ${PROJECT_NAME})

catkin_add_gtest(test_ringbuffer test/test_ringbuffer.cpp)
target_link_libraries(test_ringbuffer ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
