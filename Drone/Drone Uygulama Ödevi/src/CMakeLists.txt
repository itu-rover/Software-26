# This is the top-level CMakeLists.txt file for a catkin workspace
# catkin/cmake/toplevel.cmake is provided by ROS Noetic.
cmake_minimum_required(VERSION 3.0.2)
project(Project)
set(CATKIN_TOPLEVEL TRUE)

find_package(catkin QUIET
  NO_POLICY_SCOPE
  PATHS ${CMAKE_SOURCE_DIR}/../src/catkin/cmake
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

if(NOT catkin_FOUND)
  include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)
else()
  include(${catkin_EXTRAS_DIR}/toplevel.cmake)
endif()
