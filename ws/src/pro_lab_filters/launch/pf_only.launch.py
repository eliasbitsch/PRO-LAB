# Single-filter launch: PF only. See kf_only.launch.py for rationale.
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
            launch_arguments={'filter': 'pf'}.items(),
        ),
    ])
