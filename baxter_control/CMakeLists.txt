cmake_minimum_required(VERSION 2.8.3)
project(baxter_control)

find_package(catkin 
  REQUIRED COMPONENTS
  roscpp
  baxter_core_msgs
  cv_bridge
  image_transport
baxter_kinematics
  )
  
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  roscpp
  baxter_core_msgs
  cv_bridge
  image_transport
baxter_kinematics
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${EIGEN_INCLUDE_DIRS}
)


add_executable(baxter_enable src/baxter_enable.cpp)
target_link_libraries(baxter_enable ${catkin_LIBRARIES})

add_dependencies(baxter_enable ${catkin_EXPORTED_TARGETS})
