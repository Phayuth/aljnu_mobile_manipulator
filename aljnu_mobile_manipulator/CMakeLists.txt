cmake_minimum_required(VERSION 3.8)
project(aljnu_mobile_manipulator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(aljnu_controllers REQUIRED)
find_package(aljnu_description REQUIRED)
find_package(aljnu_path REQUIRED)

ament_package()
