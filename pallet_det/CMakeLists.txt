cmake_minimum_required(VERSION 3.0.2)
project(pallet_det)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES opencv
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs geometry_msgs
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
target_link_libraries(pallet_det_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
