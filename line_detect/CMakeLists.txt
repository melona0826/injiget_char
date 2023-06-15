cmake_minimum_required(VERSION 3.0.2)
project(line_detect)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detect_node src/detect_node.cpp)
target_link_libraries(detect_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(finish_line_detect_node src/finish_line_detect_node.cpp)
target_link_libraries(finish_line_detect_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
