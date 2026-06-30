import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = 'dexhandv2_description'
    pkg_control = 'dexhand_control'
    
    # Paths
    urdf_file = os.path.join(get_package_share_directory(pkg_desc), 'urdf/dexhandv2_right.xacro')
    world_file = os.path.join(get_package_share_directory(pkg_control), 'worlds/dexhand_grasping.world')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_control), 'config/dexhand.rviz')
    
    # Launch Configurations
    mode_arg = LaunchConfiguration('mode')
    headless_arg = LaunchConfiguration('headless')

    # 1. Gazebo Server (gzserver) with ROS plugins loaded explicitly
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    # 2. Gazebo Client (gzclient) for GUI
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # 3. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )

    # 4. Spawn Entity Node (spawns hand model in Gazebo)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'dexhand', '-topic', 'robot_description', '-z', '0.05', '-timeout', '120'],
        output='screen'
    )

    # 5. RViz2 visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 6. Advanced Tracker Node in Gazebo Mode
    advanced_tracker = Node(
        package='dexhand_control',
        executable='advanced_tracker',
        name='advanced_tracker',
        output='screen',
        parameters=[{
            'mode': mode_arg,
            'headless': headless_arg,
            'gazebo': True
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='demo',
            description='Control mode: camera or demo'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Disable webcam window'
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        advanced_tracker,
        rviz2
    ])
