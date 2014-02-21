cmake_minimum_required(VERSION 2.8.3)
project(baxter_control)

find_package(catkin 
  REQUIRED COMPONENTS
  roscpp
  baxter_core_msgs
  cv_bridge
  image_transport
  )
  
find_package(Boost REQUIRED COMPONENTS system)

#find_package(Qt4 REQUIRED COMPONENTS Qtcore QtGui)
find_package(Qt4 REQUIRED)

set( QT_USE_QTGUI TRUE)
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})
set(qt_src src/mainwindow.cpp)
set(qt_hdr include/mainwindow.h)
qt4_automoc(${qt_src})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdr})
QT4_WRAP_UI(uis_h src/mainwindow.ui)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  moveit_ros_planning_interface 
  std_msgs
  roscpp
  baxter_core_msgs
  cv_bridge
  image_transport
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${CMAKE_CURRENT_BINARY_DIR}
)

#add_executable(qtmain src/main.cpp ${uis_h} ${qt_src} ${qt_moc_srcs})
add_executable(qtmain src/main.cpp ${uis_h} ${qt_src} ${qt_moc_srcs})
target_link_libraries(qtmain ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(qtmain ${catkin_EXPORTED_TARGETS})

add_executable(baxter_enable src/baxter_enable.cpp)
target_link_libraries(baxter_enable ${catkin_LIBRARIES})
#add_dependencies(baxter_enable baxter_enable_generate_messages_cpp)

add_dependencies(baxter_enable
  ${catkin_EXPORTED_TARGETS}
  )
