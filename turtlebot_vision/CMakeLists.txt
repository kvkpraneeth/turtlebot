cmake_minimum_required(VERSION 3.5)

project(turtlebot_vision)

if(NOT CMAKE_CXX_STANDARD)

    set(CMAKE_CXX_STANDARD 17)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
)

include_directories(include)

add_executable(vision src/main.cpp)

add_library(vision_core SHARED
    src/vision.cpp
)

ament_target_dependencies(vision ${dependencies})
target_include_directories(vision
    PUBLIC include
)
target_link_libraries(vision vision_core)

ament_target_dependencies(vision_core ${dependencies})
target_include_directories(vision_core
    PUBLIC include
)
target_link_libraries(vision_core)

install(TARGETS vision_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS vision
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_libraries(vision_core)
ament_export_dependencies(${dependencies})

ament_package()

