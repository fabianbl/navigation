cmake_minimum_required (VERSION 2.8)
project(vi_turtlebot)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

cs_install()
cs_export()
