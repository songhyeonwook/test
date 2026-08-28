"""센서 계층.

  navigation        (URDF -> 센서 TF)                          <- description.launch.py
  livox_ros_driver2 (MID-360 x3 + 내장 IMU)                    <- msg_Multi_launch.py
  pointcloud_concat (3대를 base_link 기준 하나로 병합)

라이다는 3대이고 셋 다 병합에 들어간다. top 은 IMU 공급원이기도 하다.
  192.168.1.135  livox_top     IMU 공급 + cloud_in3
  192.168.2.102  livox_front   cloud_in1
  192.168.3.144  livox_rear    cloud_in2

드라이버는 xfer_format=0 (PointCloud2) 으로 낸다. 점마다 절대시각 timestamp 가
붙어 있어서 3대를 합쳐도 점 시간이 한 축 위에 놓인다.

병합 노드는 좌표 변환을 TF 로 한다. 즉 장착 기하는 URDF(mount.xacro) 한 곳에만
있고, 라이다 JSON(multi_MID360_config.json)의 extrinsic_parameter 는 전부 0 이어야
한다 - 거기에 값을 넣으면 드라이버가 점을 한 번 더 돌려서 이중 변환이 된다.

내보내는 것:
  /livox_merge/merged_pointcloud         PointCloud2  -> FAST-LIO 입력
  /livox_merge/merged_pointcloud_sliced  PointCloud2  -> Nav2 costmap 장애물
  /livox/imu_192_168_1_135               Imu          -> FAST-LIO 입력 (livox_top 내장)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 병합 노드의 입출력. 파라미터는 bringup/params/livox_merge.yaml 에 있다.
CONCAT_REMAPPINGS = [
    ('cloud_in1', '/livox/lidar_192_168_2_102'),          # livox_front
    ('cloud_in2', '/livox/lidar_192_168_3_144'),          # livox_rear
    ('cloud_in3', '/livox/lidar_192_168_1_135'),          # livox_top
    ('cloud_out', '/livox_merge/merged_pointcloud'),
    ('cloud_out_sliced', '/livox_merge/merged_pointcloud_sliced'),
]


def generate_launch_description():
    nav_dir = get_package_share_directory('navigation')
    livox_dir = get_package_share_directory('livox_ros_driver2')
    bringup_dir = get_package_share_directory('bringup')

    merge_params = os.path.join(bringup_dir, 'params', 'livox_merge.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_dir, 'launch_ROS2', 'msg_Multi_launch.py'))),
        Node(
            package='pointcloud_concatenate_ros2',
            executable='pointcloud_concat_node',
            name='livox_merge',
            output='screen',
            parameters=[merge_params],
            remappings=CONCAT_REMAPPINGS),
    ])
