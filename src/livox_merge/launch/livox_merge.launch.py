from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_merge')
    config_file = os.path.join(pkg_share, 'config', 'livox_merge_config.yaml')
    rviz_file = "/home/test/ws_livox/src/livox_merge/rviz/livox_merge.rviz"

    return LaunchDescription([
        Node(
            package='livox_merge',
            executable='merge_lidar_node',
            name='merge_lidar_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        )
    ])
