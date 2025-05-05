cmake_minimum_required(VERSION 3.8)
project(aljnu_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_executable(position_controller src/position_controller.cpp src/main_position.cpp)
ament_target_dependencies(position_controller rclcpp geometry_msgs nav_msgs)

add_executable(backstepping_controller src/backstepping_controller.cpp src/main_trajectory.cpp)
ament_target_dependencies(backstepping_controller rclcpp geometry_msgs nav_msgs)

add_executable(joystick_controller src/joystick_controller.cpp src/main_joystick.cpp)
ament_target_dependencies(joystick_controller rclcpp rclcpp_lifecycle geometry_msgs)

install(TARGETS
  position_controller
  backstepping_controller
  joystick_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()
