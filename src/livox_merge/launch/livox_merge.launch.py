"""front/rear 라이다 병합 노드 + 상단 라이다 PointCloud2 변환.

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
    top_config_file = os.path.join(pkg_share, 'config', 'livox_top_config.yaml')

    return LaunchDescription([
        Node(
            package='livox_merge',
            executable='merge_lidar_node',
            name='merge_lidar_node',
            output='screen',
            parameters=[config_file],
        ),
        # 상단 라이다(livox_top)는 병합에 안 들어가지만 포인트는 봐야 해서,
        # 같은 노드를 라이다 1대짜리로 하나 더 띄워 CustomMsg -> PointCloud2 만 한다.
        # -> /livox_top/pointcloud (frame: livox_top). 병합 경로는 안 건드린다.
        Node(
            package='livox_merge',
            executable='merge_lidar_node',
            name='livox_top_node',
            output='screen',
            parameters=[top_config_file],
        ),
    ])
