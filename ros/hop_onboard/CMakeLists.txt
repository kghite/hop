cmake_minimum_required(VERSION 3.0.2)
project(hop_onboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hindbrain src/hindbrain.cpp)
target_link_libraries(hindbrain ${catkin_LIBRARIES})