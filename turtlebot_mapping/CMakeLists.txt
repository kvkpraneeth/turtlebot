cmake_minimum_required(VERSION 3.0.2)

project(turtlebot_mapping)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    sensor_msgs
    tf2
)

catkin_package()
