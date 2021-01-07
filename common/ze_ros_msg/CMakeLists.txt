cmake_minimum_required(VERSION 3.2)
project("ze_ros_msg" CXX)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
