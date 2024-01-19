from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seal_ros_nodes',
            executable='seal_client_node',
            name='seal_client_node',
            parameters=[{'config_file': 'path/to/encryption_params_bfv.yaml'}]
        )
    ])

