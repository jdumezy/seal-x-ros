from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='seal_x_ros', 
			executable='sxr_server_node',
			output='screen',
		),
	])

