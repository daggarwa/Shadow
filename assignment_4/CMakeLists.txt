cmake_minimum_required(VERSION 2.8.3)
project(a53094896_assignment_4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MvK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a53094896_assignment_4
#  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motion_node src/motion_node.cpp)
target_link_libraries(motion_node ${catkin_LIBRARIES})
add_dependencies(motion_node a53094896_assignment_4_generate_messages_cpp)

add_executable(motion_mode_keyboard src/motion_mode_keyboard.cpp)
target_link_libraries(motion_mode_keyboard ${catkin_LIBRARIES})
add_dependencies(motion_mode_keyboard a53094896_assignment_4_generate_messages_cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

