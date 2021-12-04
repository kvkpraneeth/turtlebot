cmake_minimum_required(VERSION 3.0.2)

project(turtlebot_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    geometry_msgs
    nav_msgs
    ecl_build
    ecl_console
    ecl_mobile_robot
    ecl_converters
    ecl_devices
    ecl_geometry
    ecl_sigslots
    ecl_time
    ecl_command_line
    kobuki_driver
    tf2
    sensor_msgs
    tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(turtlebot_interface src/interface.cpp)
target_link_libraries(turtlebot_interface ${catkin_LIBRARIES})

add_executable(interface src/main.cpp)
target_link_libraries(interface turtlebot_interface ${catkin_LIBRARIES})
add_dependencies(interface turtlebot_interface)