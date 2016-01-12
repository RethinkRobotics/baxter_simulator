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
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
if(${YAML_CPP_VERSION} VERSION_LESS 0.5)
  add_definitions(-DDEPRECATED_YAML_CPP_VERSION)
endif()

catkin_package(
  CATKIN_DEPENDS
  controller_interface
  control_toolbox
  effort_controllers
  baxter_core_msgs
  DEPENDS yaml-cpp
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
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  baxter_core_msgs_gencpp
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
