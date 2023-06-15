cmake_minimum_required(VERSION 3.0.2)
project(place_move)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(place_node src/place_node.cpp)
target_link_libraries(place_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
