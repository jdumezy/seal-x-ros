cmake_minimum_required(VERSION 3.8)
project(dummy_msg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(data_pub src/data_pub.cpp)
ament_target_dependencies(data_pub rclcpp std_msgs)

add_executable(data_sub src/data_sub.cpp)
ament_target_dependencies(data_sub rclcpp std_msgs)

install(TARGETS
  data_pub
  data_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

