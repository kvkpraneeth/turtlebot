cmake_minimum_required(VERSION 3.5)

project(turtlebot_control)

if(NOT CMAKE_CXX_STANDARD)

    set(CMAKE_CXX_STANDARD 17)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    nav_msgs
)

include_directories(include)

add_executable(control src/main.cpp)

add_library(control_core SHARED
    src/control.cpp
)

ament_target_dependencies(control ${dependencies})
target_include_directories(control
    PUBLIC include
)
target_link_libraries(control control_core)

ament_target_dependencies(control_core ${dependencies})
target_include_directories(control_core
    PUBLIC include
)
target_link_libraries(control_core)

install(TARGETS control_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS control
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_libraries(control_core)
ament_export_dependencies(${dependencies})

ament_package()

