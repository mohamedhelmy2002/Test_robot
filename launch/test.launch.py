import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Declare 'use_sim_time' so you can override at launch
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to your package's share directory
    pkg_path = get_package_share_directory('test_robot')  # <-- your package name
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_.urdf.xacro')

    # Process the Xacro file into URDF XML
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
         # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
        # Joint State Publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            #arguments=['-d', os.path.join(pkg_path, 'rviz', 'robot.rviz')],
            output='screen'
        )
    ])