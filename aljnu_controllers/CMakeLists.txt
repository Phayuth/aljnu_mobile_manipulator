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
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(ament_index_cpp REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(position_controller src/position_controller.cpp src/main_position.cpp)
ament_target_dependencies(position_controller rclcpp geometry_msgs nav_msgs)

add_executable(backstepping_controller src/backstepping_controller.cpp src/main_trajectory.cpp)
ament_target_dependencies(backstepping_controller rclcpp geometry_msgs nav_msgs)

add_executable(joystick_controller src/joystick_controller.cpp src/main_joystick.cpp)
ament_target_dependencies(joystick_controller rclcpp rclcpp_lifecycle geometry_msgs)

add_executable(admittance_controller src/admittance_controller.cpp src/main_admittance.cpp)
ament_target_dependencies(admittance_controller rclcpp rclcpp_lifecycle geometry_msgs tf2_eigen kdl_parser urdf orocos_kdl Eigen3 tf2_eigen_kdl)

install(TARGETS
  position_controller
  backstepping_controller
  joystick_controller
  admittance_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kdlsolver test/test_kdlsolver.cpp src/kdlsolver.cpp)
  ament_target_dependencies(test_kdlsolver kdl_parser urdf orocos_kdl Eigen3 ament_index_cpp)
endif()

ament_package()
