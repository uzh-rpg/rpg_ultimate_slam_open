project(imp_core)
cmake_minimum_required(VERSION 2.8.0)

if(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)
  cmake_policy(SET CMP0054 OLD)
endif(${CMAKE_MAJOR_VERSION} VERSION_GREATER 3.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

set(HEADERS
  include/imp/core/macros.hpp
  include/imp/core/types.hpp

  include/imp/core/size.hpp
  include/imp/core/roi.hpp

  include/imp/core/pixel.hpp
  include/imp/core/pixel_enums.hpp
  include/imp/core/memory_storage.hpp
  include/imp/core/linearmemory_base.hpp
  include/imp/core/linearmemory.hpp
  include/imp/core/image_header.hpp
  include/imp/core/image_base.hpp
  include/imp/core/image.hpp
  include/imp/core/image_raw.hpp
  include/imp/core/image_defs.hpp
)

set(SOURCES
  src/linearmemory.cpp
  src/image_raw.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

###
### GTEST
###

catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/size_test.cpp
  test/roi_test.cpp
  test/vec_test.cpp
  test/linearmemory_test.cpp
  test/test_copy.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)

catkin_add_gtest(test_image test/test_image.cpp)
target_link_libraries(test_image ${PROJECT_NAME})

cs_install()
cs_export()

