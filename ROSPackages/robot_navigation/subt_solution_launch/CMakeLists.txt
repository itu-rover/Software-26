cmake_minimum_required(VERSION 2.8.3)
project(subt_solution_launch)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

find_package(Boost REQUIRED)

catkin_package()

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_link_flattener
  src/base_link_costmap_projector.cpp
)

add_executable(point_cloud_filter
  src/pc_filter.cpp
)

target_link_libraries(base_link_flattener
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_filter
  ${catkin_LIBRARIES}
)

install(TARGETS
  base_link_flattener
  point_cloud_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS scripts/simple_solution.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
