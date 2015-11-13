cmake_minimum_required(VERSION 2.8.3)
project(team_assignment_5)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
  message_generation
)

add_service_files(
  FILES
  SerialComm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(create_driver src/create_driver.cpp)
target_link_libraries(create_driver ${catkin_LIBRARIES})
add_dependencies(create_driver team_assignment_5_generate_messages_cpp)

add_executable(create_controller src/create_controller.cpp)
target_link_libraries(create_controller ${catkin_LIBRARIES})
add_dependencies(create_controller team_assignment_5_generate_messages_cpp)
