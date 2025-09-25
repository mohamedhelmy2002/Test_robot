from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro, os


def generate_launch_description():
    # Use sim time by default
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to your package and xacro file
    pkg_path = get_package_share_directory('test_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    

    # Publish robot_description param + tf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    

    # RViz with sim time
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        #parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        #joint_state_gui,
        robot_state_publisher,
        rviz,

        
    ])
