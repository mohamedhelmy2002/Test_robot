from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('test_robot').find('test_robot')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_.urdf.xacro'])

    # Use a simpler Command approach without FindExecutable
    robot_description_content = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),
        Node(
            package='joint_state_publisher_gui', 
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])