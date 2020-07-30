cmake_minimum_required(VERSION 3.0.2)
project(hop_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mission_controller src/mission_controller.cpp)
target_link_libraries(mission_controller ${catkin_LIBRARIES})