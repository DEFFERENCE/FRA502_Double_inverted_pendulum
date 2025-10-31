#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
    
def generate_launch_description():
    
    # Get use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    pkg = get_package_share_directory('hexapod_description')
    rviz_path = os.path.join(pkg,'config','display.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    path_description = os.path.join(pkg,'robot','visual','hexapod.xacro')
    
    # Use Command with ParameterValue for proper parsing
    robot_description_content = Command(['xacro ', path_description])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    parameters = [
        {'robot_description': robot_description},
        {'use_sim_time': use_sim_time}
    ]
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=parameters
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        rviz,
        robot_state_publisher,
        joint_state_publisher_gui
    ])