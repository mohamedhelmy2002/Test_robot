from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro, os


def generate_launch_description():
    # Use sim time by default
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to your package and xacro file
    pkg_path = get_package_share_directory('test_robot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    #"""
    # Start Gazebo with ROS interface
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )
    #"""
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
    
    # Spawn the robot in Gazebo — let’s use the param instead of topic
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )
    """
    # GUI for moving joints (optional)
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    """
    # RViz with sim time
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        #parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        #joint_state_gui,
        robot_state_publisher,
        rviz,
        gazebo,
        spawn_robot
        
    ])
