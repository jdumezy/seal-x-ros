# SEAL-X-ROS

## Overview
SEAL-X-ROS is a work in progress project to integrates the Microsoft SEAL (Simple Encrypted Arithmetic Library) with ROS2 (Robot Operating System), providing a framework for implementing fully homomorphic encryption in ROS-based applications. This project was developed as part of my final year project for my Master of engineering.

The package is built to share data between a client node (encryption and decryption) and a server node (computations on encrypted data) using services. As of right it is just a proof of concept and I have not yet added all of the functionality that I have planned.

## Prerequisites
- ROS 2 (tested with Humble)
- Microsoft SEAL 4.1
- C++17 or higher

## Installation
1. Install Microsoft SEAL following the instructions at [SEAL GitHub](https://github.com/microsoft/SEAL).
2. Clone this repository into your ROS workspace (e.g., `ros2_ws/src/`).
   ```bash
   git clone https://github.com/jdumezy/seal-x-ros.git
   ```
3. Build your ROS workspace.
   ```bash
   colcon build --packages-select seal_x_ros
   ```

## Usage
Open two terminals in your ROS2 workspace and source ROS2:
```bash
source /opt/ros/humble/setup.zsh
```
Then run the `setup` script:
```bash
source install/setup.zsh
```
Finally, launch a client and a server node, one in each terminal:
```bash
ros2 run seal_x_ros sxr_client_node
```
```bash
ros2 run seal_x_ros sxr_server_node
```

## Contributing
Contributions to the SEAL-ROS Bridge are welcome. Please submit a pull request or open an issue for any features, bugs, or improvements.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
