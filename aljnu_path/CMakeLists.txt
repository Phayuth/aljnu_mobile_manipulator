cmake_minimum_required(VERSION 3.8)
project(aljnu_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

add_executable(bezier src/bezier.cpp)
ament_target_dependencies(bezier rclcpp geometry_msgs nav_msgs visualization_msgs trajectory_msgs interactive_markers)

install(TARGETS
  bezier
  DESTINATION lib/${PROJECT_NAME})

ament_package()
