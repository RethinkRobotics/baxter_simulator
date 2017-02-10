cmake_minimum_required(VERSION 2.8.3)
project(baxter_sim_hardware)

find_package(catkin
  REQUIRED COMPONENTS
  std_msgs
  roscpp
  baxter_core_msgs
  cv_bridge
  image_transport
  baxter_sim_kinematics
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    baxter_core_msgs
    cv_bridge
    image_transport
    baxter_sim_kinematics
  DEPENDS
    OpenCV
    EIGEN3
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(baxter_emulator src/baxter_emulator.cpp)
target_link_libraries(baxter_emulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(baxter_emulator ${catkin_EXPORTED_TARGETS})

install(
  TARGETS baxter_emulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY config images include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
