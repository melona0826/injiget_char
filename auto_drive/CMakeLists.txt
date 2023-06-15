cmake_minimum_required(VERSION 3.0.2)
project(auto_drive)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_drive_node src/line_drive_node.cpp)
target_link_libraries(line_drive_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
