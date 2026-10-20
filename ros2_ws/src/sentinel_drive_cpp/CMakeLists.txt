cmake_minimum_required(VERSION 3.8)
project(sentinel_drive_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sentinel_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(cmd_vel_serial src/cmd_vel_serial.cpp)
ament_target_dependencies(cmd_vel_serial
  rclcpp std_msgs geometry_msgs sentinel_msgs)

add_executable(wheel_slip_detector src/wheel_slip_detector.cpp)
ament_target_dependencies(wheel_slip_detector rclcpp std_msgs sentinel_msgs)

add_executable(wheel_odometry src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry rclcpp nav_msgs sentinel_msgs)
target_link_libraries(wheel_odometry Eigen3::Eigen)

install(TARGETS
  cmd_vel_serial
  wheel_slip_detector
  wheel_odometry
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
