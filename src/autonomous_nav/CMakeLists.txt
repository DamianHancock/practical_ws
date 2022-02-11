cmake_minimum_required(VERSION 2.8.3)
project(autonomous_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros101
#  CATKIN_DEPENDS roscpp rospy std_msgs actionlib move_base_msgs roslib
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(traverse_goals src/traverse_goals.cpp src/nav_util.cpp)

target_link_libraries(traverse_goals ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/traverse_goals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

