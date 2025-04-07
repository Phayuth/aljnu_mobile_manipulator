cmake_minimum_required(VERSION 3.5)
project(rcontrollers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(position_controller src/position_controller.cpp)
ament_target_dependencies(position_controller rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS
  position_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()