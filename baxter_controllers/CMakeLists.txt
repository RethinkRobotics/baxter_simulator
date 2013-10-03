cmake_minimum_required(VERSION 2.8.3)
project(baxter_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  control_toolbox
  effort_controllers
  baxter_msgs
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  controller_interface
  control_toolbox
  effort_controllers
  baxter_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_library(${PROJECT_NAME}
  src/baxter_position_controller.cpp
  src/baxter_velocity_controller.cpp
  src/baxter_gripper_controller.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # wait for msgs to be generated first

