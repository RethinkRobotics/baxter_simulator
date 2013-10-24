cmake_minimum_required(VERSION 2.8.3)
project(baxter_gazebo)

find_package(catkin 
  REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    gazebo_ros_control
    controller_manager    
    baxter_msgs 
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_gazebo
  CATKIN_DEPENDS 
    baxter_msgs 
    gazebo_ros_control
    roscpp
    controller_manager_msgs
  DEPENDS 
    gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Library
add_library(baxter_gazebo_ros_control src/baxter_gazebo_ros_control_plugin.cpp)
#message(status "${catkin_LIBRARIES} \n\n\n\n\n\n\n\n ")
target_link_libraries(baxter_gazebo_ros_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} gazebo_ros_control) # \todo remove gazebo_ros_control here
add_dependencies(baxter_gazebo_ros_control ${catkin_EXPORTED_TARGETS}) # don't build until baxter_msgs is done



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baxter_gazebo baxter_gazebo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
