cmake_minimum_required(VERSION 2.8.3)
project(circlecontrol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(circlecontrol 
  src/circlecontrol.cpp
)
target_link_libraries(circlecontrol
  ${catkin_LIBRARIES}
)

