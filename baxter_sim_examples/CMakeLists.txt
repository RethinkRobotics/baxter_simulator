cmake_minimum_required(VERSION 2.8.3)
project(baxter_sim_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/ik_pick_and_place_demo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

