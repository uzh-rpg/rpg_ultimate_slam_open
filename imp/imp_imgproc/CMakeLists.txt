project(imp_imgproc)
cmake_minimum_required(VERSION 2.8.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/imp/imgproc/image_pyramid.hpp
  include/imp/imgproc/half_sample.hpp
  include/imp/imgproc/draw.hpp
)

set(SOURCES
  src/image_pyramid.cpp
  src/half_sample.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

##########
# GTESTS #
##########
catkin_add_gtest(test_image_pyramid test/test_image_pyramid.cpp)
target_link_libraries(test_image_pyramid ${PROJECT_NAME})

catkin_add_gtest(test_draw test/test_draw.cpp)
target_link_libraries(test_draw ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

