cmake_minimum_required(VERSION 3.5)

project(ros2_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
    Eigen3
	rclcpp
	kobuki_core
	geometry_msgs
	realtime_tools
	nav_msgs
)

set(INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    include
)

add_library(interface_core SHARED src/interface.cpp)
ament_target_dependencies(interface_core ${dependencies})
target_include_directories(interface_core PUBLIC include)

add_executable(interface src/main.cpp)
ament_target_dependencies(interface ${dependencies})
target_include_directories(interface PUBLIC include)
target_link_libraries(interface interface_core)

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS interface
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS interface_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

endif()


ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(interface_core)
ament_export_dependencies(${dependencies})
ament_package()