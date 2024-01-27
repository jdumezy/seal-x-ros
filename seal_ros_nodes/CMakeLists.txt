cmake_minimum_required(VERSION 3.8)
project(seal_ros_nodes)

# Standard set of compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SEAL 4.1 REQUIRED)
find_package(seal_msgs REQUIRED)

# Specify where to find the header files
include_directories(include)

# Define library targets
add_library(seal_parms_and_keys src/seal_parms_and_keys.cpp)
add_library(seal_encryptor src/seal_encryptor.cpp)
add_library(seal_decryptor src/seal_decryptor.cpp)
add_library(seal_evaluator src/seal_evaluator.cpp)
add_library(sxr_lib src/sxr_lib.cpp)

# Define executables for the nodes
add_executable(seal_client_node src/seal_client_node.cpp)
add_executable(seal_server_node src/seal_server_node.cpp)

ament_target_dependencies(seal_client_node
	rclcpp
	SEAL
	std_msgs
	seal_msgs
)

ament_target_dependencies(seal_server_node
	rclcpp
	SEAL
	std_msgs
	seal_msgs
)

target_link_libraries(seal_parms_and_keys SEAL::seal sxr_lib)
target_link_libraries(seal_encryptor SEAL::seal sxr_lib)
target_link_libraries(seal_decryptor SEAL::seal sxr_lib)
target_link_libraries(seal_evaluator SEAL::seal sxr_lib)
target_link_libraries(sxr_lib SEAL::seal)

target_link_libraries(seal_client_node
	seal_parms_and_keys
	seal_encryptor
	seal_decryptor
	SEAL::seal
)

target_link_libraries(seal_server_node
	seal_evaluator
	sxr_lib
	SEAL::seal
)

# Install targets
install(TARGETS
  seal_client_node
  seal_server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

