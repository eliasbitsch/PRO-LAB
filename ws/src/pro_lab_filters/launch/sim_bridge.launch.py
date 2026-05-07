"""Bridge + robot spawn + Nav2 for a gz sim instance already running natively in WSL.

Assumes:
  - gz sim is already running in WSL with warehouse.sdf
  - GZ_PARTITION and ROS_DOMAIN_ID match between host and container
  - This launch file runs INSIDE the container

Spawns a TurtleBot4 into the running Gazebo, bridges key topics,
starts robot_state_publisher, optionally Nav2.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pro_lab_filters')
    tb4_desc = FindPackageShare('nav2_minimal_tb4_description')
    nav2_bringup = FindPackageShare('nav2_bringup')

    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    use_nav2 = LaunchConfiguration('nav2')
    use_rviz = LaunchConfiguration('rviz')
    world_name = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Launch Nav2 stack'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('world_name', default_value='warehouse',
                              description='Name of the gz world (for topic prefixes)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Robot URDF (xacro) from nav2_minimal_tb4_description
    urdf_xacro = PathJoinSubstitution(
        [tb4_desc, 'urdf', 'standard', 'turtlebot4.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro ', urdf_xacro])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ros_gz_bridge using YAML config
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    # Spawn robot into running gz sim
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot4',
            '-topic', '/robot_description',
            '-x', x, '-y', y, '-z', '0.0', '-Y', yaw,
        ],
    )

    nav2_map = PathJoinSubstitution(
        [FindPackageShare('nav2_minimal_tb4_sim'), 'maps', 'warehouse.yaml'])

    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup, 'launch', 'bringup_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'map': nav2_map,
                'autostart': 'true',
            }.items(),
        ),
    ], condition=IfCondition(use_nav2))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [nav2_bringup, 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declare_args + [
        robot_state_publisher,
        bridge,
        spawn,
        nav2,
        rviz,
    ])
