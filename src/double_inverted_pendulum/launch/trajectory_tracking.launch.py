#!/usr/bin/env python3
"""
Launch file for Trajectory Tracking Controller
Implements Direct Collocation trajectory with time-varying LQR feedback
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for trajectory tracking controller"""
    
    # Get package share directory
    pkg_share = FindPackageShare('double_inverted_pendulum')
    
    # Path to parameters file
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'trajectory_tracking_params.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo)'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to trajectory tracking parameter file'
    )
    
    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='2.0',
        description='Delay before starting trajectory execution (seconds)'
    )
    
    # Trajectory Tracking Controller Node
    trajectory_tracking_node = Node(
        package='double_inverted_pendulum',
        executable='trajectory_tracking_controller.py',
        name='trajectory_tracking_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'startup_delay': LaunchConfiguration('startup_delay')
            }
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        startup_delay_arg,
        trajectory_tracking_node,
    ])