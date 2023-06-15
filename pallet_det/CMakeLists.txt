cmake_minimum_required(VERSION 3.0.2)
project(pallet_det)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 LIBRARIES opencv
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs geometry_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pallet_det_node src/pallet_det_node.cpp)
target_link_libraries(pallet_det_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
