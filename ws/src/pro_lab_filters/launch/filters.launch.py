from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = {'frame_id': 'odom'}
    return LaunchDescription([
        Node(package='pro_lab_filters', executable='kf_node',
             name='kf_node', output='screen', parameters=[common]),
        Node(package='pro_lab_filters', executable='ekf_node',
             name='ekf_node', output='screen', parameters=[common]),
        Node(package='pro_lab_filters', executable='pf_node',
             name='pf_node', output='screen', parameters=[common]),
    ])
