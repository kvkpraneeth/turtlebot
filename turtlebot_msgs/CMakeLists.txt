cmake_minimum_required(VERSION 3.0.2)

project(turtlebot_msgs)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

add_service_files(
  FILES
  Goal.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_runtime)
