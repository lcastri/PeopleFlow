cmake_minimum_required(VERSION 2.8.3)
project(hrisim_prediction_srvs)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Declare the service files
add_service_files(
  FILES
  GetRiskMap.srv
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
