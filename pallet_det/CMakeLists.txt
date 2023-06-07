cmake_minimum_required(VERSION 3.0.2)
project(pallet_det)

<<<<<<< HEAD
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
=======
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
<<<<<<< HEAD
  std_msgs
=======
  geometry_msgs
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42
)

find_package(OpenCV REQUIRED)

catkin_package(
<<<<<<< HEAD
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
=======
 INCLUDE_DIRS include
 LIBRARIES opencv
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs geometry_msgs
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pallet_det_node src/pallet_det_node.cpp)
<<<<<<< HEAD
target_link_libraries(pallet_det_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
=======
target_link_libraries(pallet_det_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
>>>>>>> 30531393cab546a6c417a767c67c5ba65382bd42
