# Single-filter launch: KF only.
# Thin wrapper around wrong_init_experiment.launch.py with filter:=kf.
# Use this for isolated KF testing (no EKF / PF clutter in topics & RViz).
#
# Same launch args as the master:
#   ros2 launch pro_lab_filters kf_only.launch.py scenario:=offset_5m duration_s:=60
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('pro_lab_filters')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'wrong_init_experiment.launch.py')),
            launch_arguments={'filter': 'kf'}.items(),
        ),
    ])
