cmake_minimum_required(VERSION 3.0.2)

project(turtlebot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    geometry_msgs
    nav_msgs
    fiducial_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(turtlebot_control src/control.cpp)
target_link_libraries(turtlebot_control ${catkin_LIBRARIES})

add_executable(follow src/main.cpp)
target_link_libraries(follow turtlebot_control ${catkin_LIBRARIES})
add_dependencies(follow turtlebot_control)