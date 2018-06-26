##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(baxter_sim_io)

##############################################################################
# Catkin
##############################################################################

SET(CMAKE_CXX_FLAGS "-fPIC")
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
#find_package(catkin REQUIRED COMPONENTS qt_build roscpp baxter_core_msgs)

set(CMAKE_AUTOMOC ON)
find_package(catkin REQUIRED COMPONENTS roscpp baxter_core_msgs)
find_package(Qt5Widgets REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS
  baxter_core_msgs
  INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
)


MESSAGE(STATUS ${Qt5Widgets_INCLUDE_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in manifest.xml

##############################################################################
# Sections
##############################################################################
#include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/baxter_sim_io/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(baxter_sim_io ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(baxter_sim_io Qt5::Widgets ${ROS_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS baxter_sim_io
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include resources ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
