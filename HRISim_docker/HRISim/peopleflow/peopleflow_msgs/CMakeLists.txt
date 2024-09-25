cmake_minimum_required(VERSION 2.8.3)
project(peopleflow_msgs)


find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg
  FILES
  pfAgent.msg
  pfAgents.msg
  WPPeopleCounter.msg
  WPPeopleCounters.msg
  Time.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


#Declare package run-time dependencies
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime std_msgs geometry_msgs

)
