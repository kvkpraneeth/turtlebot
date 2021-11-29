cmake_minimum_required(VERSION 3.0.2)

project(turtlebot_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    sensor_msgs
    tf2
    realsense2_camera
    realsense2_description
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(turtlebot_vision src/vision.cpp)
target_link_libraries(turtlebot_vision ${catkin_LIBRARIES})

add_executable(vision src/main.cpp)
target_link_libraries(vision turtlebot_vision ${catkin_LIBRARIES})
add_dependencies(vision turtlebot_vision)