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

## Warning: No Guarantee on Security

This ROS package is a work in progress and is provided for research and experimentation purposes only. It should not be used in production systems or applications where security is a critical concern.

There are no guarantees on the security of this library, and it may contain vulnerabilities, weaknesses, or errors that could compromise the confidentiality or integrity of your data. You are using this library at your own risk, and the developers and maintainers of this library cannot be held responsible for any security breaches or data loss that may occur as a result of its use.

Before using this library, please carefully review the source code, documentation, and any associated warnings or disclaimers. It is strongly recommended that you consult with cryptography experts and conduct thorough security assessments before considering it for any security-sensitive applications.

**DO NOT USE THIS LIBRARY IN PRODUCTION OR MISSION-CRITICAL SYSTEMS.**

By using this library, you acknowledge and accept the risks associated with its use, and you agree to hold the developers and maintainers harmless from any liabilities or damages that may arise from its use. If you have any concerns about the security of this library or encounter any issues, please report them to the project's issue tracker.

Please use this library responsibly and in a controlled, non-critical environment for research and experimentation purposes only.

## Contributing
Contributions to the SEAL-ROS Bridge are welcome. Please submit a pull request or open an issue for any features, bugs, or improvements.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
