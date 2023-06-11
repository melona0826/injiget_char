cmake_minimum_required(VERSION 3.0.2)
project(classification)

find_package(catkin REQUIRED COMPONENTS
  rospy

)

catkin_package(

)

include_directories(

)


catkin_install_python(PROGRAMS
  scripts/classification_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
