import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = 'dexhandv2_description'
    pkg_control = 'dexhand_control'
    
    urdf_file = os.path.join(get_package_share_directory(pkg_control), 'urdf/upper_body.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_control), 'config/dexhand.rviz')

    # Launch Configurations
    mode_arg = LaunchConfiguration('mode')
    headless_arg = LaunchConfiguration('headless')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'mode',
            default_value='camera',
            description='Execution mode: camera (uses MediaPipe) or demo (automated motions)'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Whether to disable OpenCV popup window'
        ),
      
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # RViz2 with pre-loaded configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Advanced Tracker Node
        Node(
            package='dexhand_control',
            executable='advanced_tracker',
            name='advanced_tracker',
            output='screen',
            parameters=[{
                'mode': mode_arg,
                'headless': headless_arg
            }]
        )
    ])
