cmake_minimum_required(VERSION 2.8.3)
project(baxter_sim_controllers)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  controller_interface
  control_toolbox
  effort_controllers
  baxter_core_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

# Support new yaml-cpp API.
find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_library(YAML yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  controller_interface
  control_toolbox
  effort_controllers
  baxter_core_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/baxter_position_controller.cpp
  src/baxter_velocity_controller.cpp
  src/baxter_effort_controller.cpp
  src/baxter_head_controller.cpp
  src/baxter_gripper_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${YAML}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
