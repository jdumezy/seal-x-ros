# SEAL-ROS Bridge

## Overview
The SEAL-ROS Bridge project integrates the Microsoft SEAL (Simple Encrypted Arithmetic Library) with ROS (Robot Operating System), providing a framework for implementing fully homomorphic encryption in ROS-based applications. This project is especially useful for developers looking to enhance data security in robotic communication systems.

## Features
- Integration of SEAL FHE library with ROS.
- Custom ROS messages for encrypted data transfer.
- Example ROS nodes for encryption and decryption using SEAL.

## Prerequisites
- ROS 2 (tested with Humble)
- Microsoft SEAL Library
- C++17 or higher

## Installation
1. Clone this repository into your ROS workspace (e.g., `ros2_ws/src/`).
   ```bash
   cd ~/ros2_ws/src/
   git clone https://github.com/jdumezy/seal-ros-bridge.git
   ```
2. Install Microsoft SEAL following the instructions at [SEAL GitHub](https://github.com/microsoft/SEAL).

3. Build your ROS workspace.
   ```bash
   cd ~/ros2_ws/
   colcon build --packages-select seal_ros_nodes seal_msgs
   ```

## Usage
Describe how to run and use your ROS packages here. Include any necessary steps to launch nodes, example use cases, or command-line instructions.

## Contributing
Contributions to the SEAL-ROS Bridge are welcome. Please submit a pull request or open an issue for any features, bugs, or improvements.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
