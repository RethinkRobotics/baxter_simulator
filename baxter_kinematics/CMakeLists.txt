cmake_minimum_required(VERSION 2.8.3)
project(baxter_kinematics)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  baxter_core_msgs
  motor_control_msgs
  sensor_msgs
  tf
  tf_conversions
  kdl_parser
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  baxter_core_msgs
  motor_control_msgs
  sensor_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
 
 #add_dependencies(baxter_kinematics baxter_kinematics_generate_messages_cpp)


 #target_link_libraries(baxter_kinematics ${catkin_LIBRARIES})
add_library(arm_kinematics src/arm_kinematics.cpp)
target_link_libraries(arm_kinematics ${catkin_LIBRARIES})

add_executable(baxter_kinematics src/PositionKinematicsNode.cpp)
 target_link_libraries(baxter_kinematics ${catkin_LIBRARIES} arm_kinematics)


 add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
 )
