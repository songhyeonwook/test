"""front/rear 라이다 병합 노드.

RViz 는 여기서 띄우지 않는다. 옛 워크스페이스의 하드코딩된 rviz 경로
(/home/test/ws_livox/...)가 남아 있어 실장비 bringup 때 빈 RViz 창이 하나 더
떴었다. 시각화는 localization.launch.py 의 RViz 를 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_merge')
    config_file = os.path.join(pkg_share, 'config', 'livox_merge_config.yaml')

    return LaunchDescription([
        Node(
            package='livox_merge',
            executable='merge_lidar_node',
            name='merge_lidar_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
