"""Single-container launch: headless Gazebo + spawn + bridges + Nav2 + RViz.

Runs entirely inside the container. RViz is the only visible UI.
Composes the pieces manually (instead of tb4_simulation_launch.py which
expects world.xacro paths).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    map_yaml = LaunchConfiguration('map')

    args = [
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World name (maze|depot|warehouse)'),
        DeclareLaunchArgument('x_pose', default_value='2.12'),
        DeclareLaunchArgument('y_pose', default_value='-21.3'),
        DeclareLaunchArgument('yaw', default_value='1.57'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav2_bringup, 'maps', 'warehouse.yaml')),
    ]

    # 1. Headless Gazebo server
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '3',
             PathJoinSubstitution([sim_dir, 'worlds', [world, '.sdf']])],
        output='screen',
    )

    # 2. Robot_state_publisher + bridges + spawner (from nav2_minimal_tb4_sim)
    # This launch also starts robot_state_publisher via the URDF on /robot_description
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare('nav2_minimal_tb4_description'),
         'urdf', 'standard', 'turtlebot4.urdf.xacro'])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_xacro]),
        }],
    )

    # 3. Spawn TB4 into Gazebo (delayed so gz server is ready)
    spawn = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')),
            launch_arguments={
                'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw,
            }.items(),
        )
    ])

    # 4. Nav2 stack (localization + nav, no simulator)
    nav2 = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map': map_yaml,
                'autostart': 'true',
            }.items(),
        )
    ])

    # 5. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Foxglove WebSocket bridge (port 8765). Open Foxglove Studio (web or
    # desktop) and connect to ws://localhost:8765, then load
    # config/foxglove_layout.json from this package for a pre-built layout.
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'port': 8765,
            'address': '0.0.0.0',
            'max_qos_depth': 25,
            'send_buffer_limit': 10000000,
        }],
        output='screen',
    )

    return LaunchDescription(args + [gz_server, robot_state_pub, spawn, nav2,
                                     rviz, foxglove])
