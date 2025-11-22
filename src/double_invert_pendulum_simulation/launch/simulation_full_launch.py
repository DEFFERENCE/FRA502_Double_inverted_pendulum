#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package shares
    desc_pkg = get_package_share_directory('double_invert_pendulum_description')
    sim_pkg = get_package_share_directory('double_invert_pendulum_simulation')

    # Gazebo resource paths
    install_share_dir = os.path.dirname(desc_pkg)
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description
    xacro_file = PathJoinSubstitution([
        FindPackageShare('double_invert_pendulum_description'), 
        'robot', 'visual', 'double_invert_pendulum.xacro'
    ])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 1 empty.sdf'}.items()
    )

    # Robot state publisher
    state_pub = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_param},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Spawn entity - RAISED TO 0.5m TO MATCH XACRO FIX
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create', 
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'double_invert_pendulum',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'  # world link spawns at 0, base_link is at +0.5 in xacro
        ]
    )

    # Controller spawners
    jsb_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_config = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', 
                            description='Use simulation clock'),
        gz_model_path,
        ign_model_path,
        gz_sim,
        state_pub,  # Start immediately for RViz
        spawn_entity,
        bridge,
        rviz,  # Start with state_pub, no need to wait
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[effort_spawner]
            )
        )
    ])