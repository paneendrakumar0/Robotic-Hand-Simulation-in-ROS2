import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_desc = 'dexhandv2_description'
    urdf_file = os.path.join(get_package_share_directory(pkg_desc), 'urdf/dexhandv2_right.xacro')

    return LaunchDescription([
      
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

   
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

       
        Node(
            package='dexhand_control',
            executable='advanced_tracker',
            name='advanced_tracker',
            output='screen'
        )
    ])
