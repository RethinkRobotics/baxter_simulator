cmake_minimum_required(VERSION 2.8.3)
project(baxter_control)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  moveit_ros_planning_interface 
  std_msgs
  roscpp
  baxter_core_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

