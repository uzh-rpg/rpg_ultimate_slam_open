cmake_minimum_required(VERSION 2.8.3)
project(ze_cmake)

find_package(catkin_simple REQUIRED)
catkin_simple()

# install all included cmake files
install(DIRECTORY cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

cs_install()
cs_export(CFG_EXTRAS ze_cmake-extras.cmake)
