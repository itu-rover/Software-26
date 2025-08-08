cmake_minimum_required(VERSION 3.0.2)
project(arm23_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ros_control_boilerplate
  controller_manager
  hardware_interface
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm23_hw_main
  include/arm23_control/arm23_hw_interface.h
  src/arm23_hw_interface.cpp
  src/arm23_hw_main.cpp
)

add_dependencies(arm23_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm23_hw_main
  ${catkin_LIBRARIES}
)
