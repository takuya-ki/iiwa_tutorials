cmake_minimum_required(VERSION 2.8.3)
project(rviz_controller)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS 
    rviz
    roscpp
    cv_bridge
    image_transport
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    rviz
    roscpp
    cv_bridge
    image_transport
  DEPENDS
    rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES
  include/screen_publisher_rviz_plugin.hpp
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  src/screen_publisher_rviz_plugin.cpp
  ${MOC_FILES}
)

add_library(
  rviz_plugins
    ${SOURCE_FILES}
)
target_link_libraries(
  rviz_plugins 
    ${QT_LIBRARIES} 
    ${catkin_LIBRARIES}
)