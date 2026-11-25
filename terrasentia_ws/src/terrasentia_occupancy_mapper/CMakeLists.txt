cmake_minimum_required(VERSION 2.8.3)
project(terrasentia_occupancy_mapper)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

