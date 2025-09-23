import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('test_robot')

    return LaunchDescription([
        # 1. Launch RViz/display.launch.py with use_sim_time true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', 'display.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items() 
        ),
         # 2. Launch Gazebo (empty world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            # you can add world/verbose args here
            # launch_arguments={'world': '/path/to/world.sdf', 'verbose':'true'}.items()
        ),
        # 3. Spawn your robot into Gazebo
        Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')
                        
    ])