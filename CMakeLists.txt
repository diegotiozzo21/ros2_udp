cmake_minimum_required(VERSION 3.8)
project(ros2_udp)

#Default to C++14
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

add_executable(udp_server src/udp_server.cpp)
ament_target_dependencies(udp_server rclcpp std_msgs geometry_msgs)
add_executable(udp_client src/udp_client.cpp)
ament_target_dependencies(udp_client rclcpp std_msgs)

install(TARGETS
  udp_server
  udp_client 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
