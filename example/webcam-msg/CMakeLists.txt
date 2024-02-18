cmake_minimum_required(VERSION 3.8)
project(webcam_msg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(webcam_msg_lib src/webcam_msg_lib.cpp)

add_executable(webcam_pub src/webcam_pub.cpp)
ament_target_dependencies(webcam_pub rclcpp std_msgs)

add_executable(webcam_sub src/webcam_sub.cpp)
ament_target_dependencies(webcam_sub rclcpp std_msgs)

target_link_libraries(webcam_msg_lib ${OpenCV_LIBS})
target_link_libraries(webcam_sub webcam_msg_lib)
target_link_libraries(webcam_pub webcam_msg_lib)

install(TARGETS
  webcam_pub
  webcam_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

