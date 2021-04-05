cmake_minimum_required(VERSION 2.8.3)

project(turtlebot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    message_generation 
    geometry_msgs
)

add_service_files(FILES vel.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(turtlebot_control src/turtlebot_control.cpp)
target_link_libraries(turtlebot_control ${catkin_LIBRARIES})
add_dependencies(turtlebot_control turtlebot_gencpp)

add_executable(controller src/main.cpp)
target_link_libraries(controller turtlebot_control ${catkin_LIBRARIES})
add_dependencies(controller turtlebot_control)

