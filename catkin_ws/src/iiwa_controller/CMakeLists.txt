cmake_minimum_required(VERSION 2.8.3)
project(iiwa_controller)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS 
    geometry_msgs
    roscpp
    rospy
    std_msgs
    control_msgs
    sensor_msgs
    tf2_ros
    iiwa_ros
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    message_generation
    actionlib_msgs
    cv_bridge
    message_filters
    dynamic_reconfigure
)

add_service_files(
  FILES
    SetTargetPosition.srv
    SetTargetPose.srv
    SetTarget.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    control_msgs
    sensor_msgs
    tf2_ros
    iiwa_ros
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    message_runtime
    actionlib_msgs
    cv_bridge
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  iiwa_controller_node
    src/iiwa_controller_node.cpp
)
add_dependencies(
  iiwa_controller_node
    iiwa_msgs_generate_messages_cpp
    iiwa_controller_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  iiwa_controller_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(iiwa_tcp_move src/iiwa_tcp_move.cpp)
add_dependencies(iiwa_tcp_move iiwa_msgs_generate_messages_cpp)
target_link_libraries(iiwa_tcp_move ${catkin_LIBRARIES})

add_executable(iiwa_tcp_move_to_manipulate src/iiwa_tcp_move_to_manipulate.cpp)
add_dependencies(iiwa_tcp_move_to_manipulate iiwa_msgs_generate_messages_cpp)
target_link_libraries(iiwa_tcp_move_to_manipulate ${catkin_LIBRARIES})
