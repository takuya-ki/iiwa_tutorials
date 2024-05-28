cmake_minimum_required(VERSION 3.0.2)
project(iiwa_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  iiwa_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    iiwa_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  wiggle
    src/wiggle.cpp
)
add_dependencies(
  wiggle
    iiwa_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  wiggle
    ${catkin_LIBRARIES}
)

add_executable(
  sendmotion_loop
    src/sendmotion_loop.cpp
)
add_dependencies(
  sendmotion_loop
    iiwa_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  sendmotion_loop
    ${catkin_LIBRARIES}
)

add_executable(
  sendmotion_loop_action
    src/sendmotion_loop_action.cpp
)
add_dependencies(
  sendmotion_loop_action
    iiwa_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  sendmotion_loop_action
    ${catkin_LIBRARIES}
)

add_executable(
  sendmotion_loop_lin
    src/sendmotion_loop_lin.cpp
)
add_dependencies(
  sendmotion_loop_lin
    iiwa_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  sendmotion_loop_lin
    ${catkin_LIBRARIES}
)