cmake_minimum_required(VERSION 3.0.2)
project(verification)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_python_setup()


catkin_package(
#INCLUDE_DIRS include
#LIBRARIES verification
CATKIN_DEPENDS roscpp rospy std_msgs
#DEPENDS system_lib
)

catkin_install_python(PROGRAMS
src/robot_reachability.py
src/human_reachability.py
src/marker_pub.py
src/collision.py

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
