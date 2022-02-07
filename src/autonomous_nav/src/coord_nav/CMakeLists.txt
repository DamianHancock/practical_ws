cmake_minimum_required(VERSION 2.8.3)
project(coord_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros101
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS src/coordinate_send.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

