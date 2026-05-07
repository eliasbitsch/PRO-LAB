"""Wrong-initialization experiment launcher.

Runs the simulation + all 3 filters + truth relay + metrics + CSV logger
under a chosen scenario YAML. Optionally auto-shuts-down after `duration_s`.

Both visualizers (RViz + Foxglove bridge) are started — connect either or both.

Usage:
    ros2 launch pro_lab_filters wrong_init_experiment.launch.py \\
        scenario:=offset_5m duration_s:=60 out_dir:=/tmp/results

Available scenarios (config/scenarios/<name>.yaml):
    correct_init, offset_1m, offset_5m, wrong_yaw_pi2,
    overconfident_wrong, underconfident, kidnapped
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction, RegisterEventHandler,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression,
                                  TextSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pro_lab_filters')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')

    scenario   = LaunchConfiguration('scenario')
    duration_s = LaunchConfiguration('duration_s')
    out_dir    = LaunchConfiguration('out_dir')
    world      = LaunchConfiguration('world')
    x_pose     = LaunchConfiguration('x_pose')
    y_pose     = LaunchConfiguration('y_pose')
    yaw        = LaunchConfiguration('yaw')
    map_yaml   = LaunchConfiguration('map')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_nav2   = LaunchConfiguration('use_nav2')
    start_gz   = LaunchConfiguration('start_gz')
    filter_sel = LaunchConfiguration('filter')

    args = [
        DeclareLaunchArgument('scenario',   default_value='correct_init'),
        DeclareLaunchArgument('duration_s', default_value='0',
                              description='Auto-shutdown after N seconds. 0 = run forever.'),
        DeclareLaunchArgument('out_dir',    default_value='/tmp/pro_lab_results'),
        DeclareLaunchArgument('world',      default_value='warehouse'),
        # Defaults match nav2_minimal_tb4_sim's warehouse map origin so
        # /scan aligns with /map. Override these in scenario YAMLs (init_x
        # /init_y/init_yaw) to study wrong-init recovery.
        DeclareLaunchArgument('x_pose',     default_value='-8.00'),
        DeclareLaunchArgument('y_pose',     default_value='-0.50'),
        DeclareLaunchArgument('yaw',        default_value='0.0'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(nav2_bringup, 'maps', 'warehouse.yaml')),
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('use_nav2',     default_value='true'),
        DeclareLaunchArgument('filter',       default_value='all',
            description='Which estimator(s) to run: kf | ekf | pf | all'),
        DeclareLaunchArgument('start_gz',     default_value='false',
                              description='Start headless gz server inside the container. '
                                          'Set false when gz sim is already running natively '
                                          '(GPU mode in WSL).'),
    ]

    scenario_file = PathJoinSubstitution(
        [pkg_share, 'config', 'scenarios', [scenario, TextSubstitution(text='.yaml')]])

    # ── Simulation stack (gz + RSP + spawn) ──────────────────────────────
    # gz_server is optional: when start_gz:=false, we expect a Gazebo server
    # to be already running externally (e.g. natively in WSL with GPU access).
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '3',
             PathJoinSubstitution([sim_dir, 'worlds', [world, '.sdf']])],
        output='screen',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(start_gz),
    )

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

    spawn = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'yaw': yaw}.items(),
        )
    ])

    # ros_gz_bridge: maps gz topics (/cmd_vel, /imu, /odom, /scan, /tf, /clock)
    # to ROS so the filter nodes see them. Required whether gz runs inside the
    # container or natively in WSL.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    # ── Nav2 (optional — provides /pose via AMCL) ────────────────────────
    nav2 = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map': map_yaml,
                'autostart': 'true',
            }.items(),
            condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_nav2),
        )
    ])

    # ── Standalone map_server so /map is always published ──────────────
    # Even when Nav2 is disabled the user still wants the warehouse map as
    # the 3D-panel background, so we always run a map_server + lifecycle
    # auto-starter against the same warehouse.yaml that Nav2 uses.
    map_yaml_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'maps', 'warehouse.yaml'])
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml_path}],
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    # ── cmd_vel watchdog ────────────────────────────────────────────────
    # Foxglove's Teleop panel only publishes while a key is held. On
    # release it stops publishing — but Gazebo keeps applying the last
    # twist forever. This relay forwards /cmd_vel_in (Teleop's output) to
    # /cmd_vel and zeroes /cmd_vel after a short silence, so releasing the
    # button actually stops the robot.
    cmd_vel_watchdog = Node(
        package='pro_lab_filters',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'timeout': 0.4,
            'input_topic': '/cmd_vel_in',
            'output_topic': '/cmd_vel',
        }],
    )

    # Live "kidnap" the robot: shift+click in Foxglove -> /initialpose
    # -> robot_teleporter -> gz set_pose service -> robot jumps there.
    robot_teleporter = Node(
        package='pro_lab_filters',
        executable='robot_teleporter',
        name='robot_teleporter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_name': 'turtlebot4',
            'world_name': world,
            'spawn_z': 0.05,
            'gz_partition': 'prolab',
        }],
    )

    # Landmark detector — assignment requires a self-defined landmark.
    # Three vertical posts at known map coords; node simulates noisy
    # range/bearing observations from /ground_truth/pose for filters that
    # want to consume them, plus RViz markers so the posts are visible.
    landmark_detector = Node(
        package='pro_lab_filters',
        executable='landmark_detector_node',
        name='landmark_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'landmark_ids': [1, 2, 3],
            'landmark_xs': [-5.0,  3.0, -2.0],
            'landmark_ys': [ 2.0, -2.0, -6.0],
            'range_sigma':   0.10,
            'bearing_sigma': 0.05,
            'max_range':     6.0,
            'frame_id':      'map',
            'rate_hz':       5.0,
        }],
    )

    # Drag-to-drive interactive marker (RViz-side teleop).
    teleop_marker = Node(
        package='pro_lab_filters',
        executable='teleop_marker_node',
        name='teleop_marker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
        }],
    )

    # Periodically rebroadcast /map on /map_repub so Foxglove always sees it.
    map_republisher = Node(
        package='pro_lab_filters',
        executable='map_republisher',
        name='map_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/map',
            'output_topic': '/map_repub',
            'rate_hz': 0.2,
        }],
    )

    # ── Dynamic map -> odom driven by the PF estimate (AMCL-style) ─────
    # The PF publishes its best-estimate pose in the map frame. We compute
    # map_T_odom = pf_pose * inv(odom_T_base) and broadcast it on /tf so
    # the laser scan and other map-frame visualisations stay aligned with
    # the warehouse walls even after teleporting (kidnapping).
    map_odom_tf = Node(
        package='pro_lab_filters',
        executable='map_odom_tf_publisher',
        name='map_odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic':  '/pf/pose',
            'map_frame':   'map',
            'odom_frame':  'odom',
            'base_frame':  'base_footprint',
        }],
    )

    # ── Visualizers (both, in parallel) ──────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'wrong_init.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_rviz),
    )
    # Foxglove bridge: open Foxglove Studio (web or desktop), connect to
    # ws://localhost:8765, then load config/foxglove_layout.json.
    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        parameters=[{'use_sim_time': True, 'port': 8765, 'address': '0.0.0.0',
                     'max_qos_depth': 25, 'send_buffer_limit': 10000000}],
        output='screen',
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(use_foxglove),
    )

    # ── Filters: each loads the same scenario YAML so init_x/spread/etc.
    #    are consistent across KF, EKF, PF. The `filter` launch arg gates
    #    which one(s) actually run, so single-filter testing is one CLI
    #    flag away (kf_only.launch.py / ekf_only.launch.py / pf_only.launch.py
    #    are thin wrappers around this).
    run_kf  = PythonExpression(["'", filter_sel, "' in ('kf',  'all')"])
    run_ekf = PythonExpression(["'", filter_sel, "' in ('ekf', 'all')"])
    run_pf  = PythonExpression(["'", filter_sel, "' in ('pf',  'all')"])

    kf  = Node(package='pro_lab_filters', executable='kf_node',  name='kf_node',
               parameters=[scenario_file, {'frame_id': 'odom', 'use_sim_time': True}],
               output='screen', condition=IfCondition(run_kf))
    ekf = Node(package='pro_lab_filters', executable='ekf_node', name='ekf_node',
               parameters=[scenario_file, {'frame_id': 'odom', 'use_sim_time': True}],
               output='screen', condition=IfCondition(run_ekf))
    pf  = Node(package='pro_lab_filters', executable='pf_node',  name='pf_node',
               parameters=[scenario_file, {'frame_id': 'map', 'use_sim_time': True,
                                           'publish_particles': True}],
               output='screen', condition=IfCondition(run_pf))

    # ── Truth + metrics + CSV logger ────────────────────────────────────
    truth = Node(package='pro_lab_filters', executable='truth_relay_node',
                 name='truth_relay',
                 parameters=[{'use_sim_time': True,
                              'target_frame': 'odom',
                              'source_frame': 'base_footprint',
                              'publish_hz': 20.0,
                              'topic': '/ground_truth/pose'}],
                 output='screen')
    metrics = Node(package='pro_lab_filters', executable='metrics_node',
                   name='metrics_node',
                   parameters=[{'use_sim_time': True,
                                'convergence_threshold_xy': 0.20,
                                'convergence_window_s': 2.0,
                                'filters': ['kf', 'ekf', 'pf']}],
                   output='screen')
    csv_logger = Node(package='pro_lab_filters', executable='csv_logger.py',
                      name='csv_logger',
                      parameters=[{'use_sim_time': True,
                                   'scenario': scenario,
                                   'out_dir': out_dir}],
                      output='screen')

    filters_group = TimerAction(period=10.0, actions=[
        kf, ekf, pf, truth, metrics, csv_logger,
    ])

    # ── Optional auto-shutdown after duration_s seconds ─────────────────
    def maybe_shutdown(context, *args, **kwargs):
        try:
            d = float(context.launch_configurations.get('duration_s', '0'))
        except ValueError:
            d = 0.0
        if d <= 0:
            return []
        return [TimerAction(period=d, actions=[Shutdown(reason='duration elapsed')])]

    return LaunchDescription(
        args + [gz_server, robot_state_pub, spawn, bridge, nav2,
                map_server, map_lifecycle, map_odom_tf,
                cmd_vel_watchdog, map_republisher, robot_teleporter,
                teleop_marker, landmark_detector,
                rviz, foxglove, filters_group,
                OpaqueFunction(function=maybe_shutdown)])
