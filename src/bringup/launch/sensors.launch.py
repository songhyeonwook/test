"""센서 계층.

  livox_ros_driver2 (MID-360 x2 + 내장 IMU) + 차체 static TF   <- msg_Multi_launch.py
  livox_merge       (2대를 body 기준 하나로 병합)              <- livox_merge.launch.py

드라이버는 CustomMsg 로 낸다(livox_merge 가 그걸 받는다). 병합 이후로는 전부
PointCloud2 를 쓴다.

내보내는 것:
  /livox_merge/merged_pointcloud   PointCloud2  -> FAST-LIO 입력 (ouster 포맷)
  /livox_merge/merged_livox        CustomMsg    -> 필요할 때만
  /livox/imu_192_168_1_154         Imu          -> FAST-LIO 입력 (상단 라이다 내장)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_dir = get_package_share_directory('livox_ros_driver2')
    merge_dir = get_package_share_directory('livox_merge')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_dir, 'launch_ROS2', 'msg_Multi_launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(merge_dir, 'launch', 'livox_merge.launch.py'))),
    ])
