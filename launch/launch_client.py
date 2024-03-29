from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration 	 
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='seal_x_ros', 
			executable='sxr_client_node',
			namespace='private',
			output='screen',
		),
		LogInfo(
			condition=IfCondition(LaunchConfiguration('enable_debug')),
			message="Debugging is enabled.",
		),
	])

