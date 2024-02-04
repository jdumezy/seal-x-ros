cmake_minimum_required(VERSION 3.8)
project(seal_x_ros)

# Standard set of compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(SEAL 4.1 REQUIRED)

# Specify where to find the header files
include_directories(include)

# Define library targets
add_library(sxr_parms_and_keys src/sxr_parms_and_keys.cpp)
add_library(sxr_encryptor src/sxr_encryptor.cpp)
add_library(sxr_decryptor src/sxr_decryptor.cpp)
add_library(sxr_evaluator src/sxr_evaluator.cpp)
add_library(sxr_ciphertext src/sxr_ciphertext.cpp)
add_library(sxr_lib src/sxr_lib.cpp)

# Define executables for the nodes
add_executable(sxr_client_node src/sxr_client_node.cpp)
add_executable(sxr_server_node src/sxr_server_node.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KeyExchange.srv"
  "srv/OperationRequest.srv"
  "srv/ServerMessage.srv"
)

ament_target_dependencies(sxr_client_node
	rclcpp
	std_msgs
	SEAL
)

ament_target_dependencies(sxr_server_node
	rclcpp
	std_msgs
	SEAL
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(sxr_parms_and_keys SEAL::seal sxr_lib)
target_link_libraries(sxr_encryptor SEAL::seal sxr_lib)
target_link_libraries(sxr_decryptor SEAL::seal sxr_lib)
target_link_libraries(sxr_evaluator SEAL::seal sxr_lib sxr_ciphertext)
target_link_libraries(sxr_ciphertext SEAL::seal sxr_lib)
target_link_libraries(sxr_lib SEAL::seal)

target_link_libraries(sxr_client_node
	sxr_decryptor
	sxr_encryptor
	sxr_parms_and_keys
	SEAL::seal
	"${cpp_typesupport_target}"
)

target_link_libraries(sxr_server_node
	sxr_ciphertext
	sxr_encryptor
	sxr_evaluator
	sxr_lib
	SEAL::seal
	"${cpp_typesupport_target}"
)

# Install targets
install(TARGETS
  sxr_client_node
  sxr_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

