"""센서 계층.

  description       (URDF -> body 아래 센서 TF)               <- description.launch.py
  livox_ros_driver2 (MID-360 x3 + 내장 IMU)                    <- msg_Multi_launch.py
  livox_merge       (front/rear 2대를 body 기준 하나로 병합)   <- livox_merge.launch.py

라이다는 3대다. 병합에 들어가는 건 front/rear 둘뿐이고, top 은 IMU 공급원으로만
쓴다(포인트는 현재 아무도 구독하지 않는다).
  192.168.1.135  livox_top     IMU 공급
  192.168.2.102  livox_front   merge lidar_0  -> body 기준
  192.168.3.144  livox_rear    merge lidar_1

드라이버는 CustomMsg 로 낸다(livox_merge 가 그걸 받는다). 병합 이후로는 전부
PointCloud2 를 쓴다.

내보내는 것:
  /livox_merge/merged_pointcloud   PointCloud2  -> FAST-LIO 입력 (ouster 포맷)
  /livox_merge/merged_livox        CustomMsg    -> 필요할 때만
  /livox/imu_192_168_1_135         Imu          -> FAST-LIO 입력 (livox_top 내장)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    desc_dir = get_package_share_directory('description')
    livox_dir = get_package_share_directory('livox_ros_driver2')
    merge_dir = get_package_share_directory('livox_merge')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_dir, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_dir, 'launch_ROS2', 'msg_Multi_launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(merge_dir, 'launch', 'livox_merge.launch.py'))),
    ])
