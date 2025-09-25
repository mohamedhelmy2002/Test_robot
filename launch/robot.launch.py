import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('test_robot')

    launch_display = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'display.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items() 
        )
    # Spawn the robot in Gazebo — let’s use the param instead of topic
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Start Gazebo with ROS interface
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    return LaunchDescription([
        launch_display,
        spawn_robot,
        gazebo
    ])