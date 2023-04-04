cmake_minimum_required(VERSION 3.5)
project(coppelia_command)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(xMover src/xMover.cpp)
ament_target_dependencies(xMover rclcpp std_msgs)

add_executable(xyMover src/xyMover.cpp)
ament_target_dependencies(xyMover rclcpp std_msgs)

install(TARGETS
    xMover
    xyMover
    DESTINATION lib/${PROJECT_NAME})

ament_package()
