cmake_minimum_required(VERSION 2.8.3)
project(robot_srvs)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Declare the service files
add_service_files(
  FILES
  NewTask.srv
  FinishTask.srv
  SetBattery.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)