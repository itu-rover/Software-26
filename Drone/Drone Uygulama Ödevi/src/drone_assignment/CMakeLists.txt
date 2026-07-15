cmake_minimum_required(VERSION 3.0.2)
project(drone_assignment)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_template.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
