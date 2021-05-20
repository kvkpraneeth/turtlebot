cmake_minimum_required(VERSION 3.5)

project(turtlebot)

if(NOT CMAKE_CXX_STANDARD)

    set(CMAKE_CXX_STANDARD 17)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic)

endif()

find_package(ament_cmake REQUIRED)
find_package(turtlebot_control REQUIRED)
find_package(turtlebot_description REQUIRED)

set(dependencies
    turtlebot_control
    turtlebot_description
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()

ament_export_dependencies(${dependencies})

ament_package()

