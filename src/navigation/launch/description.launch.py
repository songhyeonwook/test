"""URDF 를 읽어 TF 를 내보낸다.

  vehicle   base_link -> base_footprint -> 센서들   (사용자가 보는 모델)

map->odom, odom->base_link 은 fast_lio_localization 의 tf_2d.py 가 FAST-LIO
오도메트리 토픽을 평면화해서 낸다. 전체는 단일 체인이다.

  map ── odom ── base_link ── base_footprint ── livox_*

FAST-LIO 내부 프레임(camera_init, body)은 TF 에 올리지 않는다. body -> 차량
기준점 extrinsic 은 fast_lio_localization/config/mid360.yaml 의 ref_from_body_*
가 든다.

관절이 전부 fixed 라 joint_state_publisher 는 필요 없다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_dir = os.path.join(get_package_share_directory('navigation'), 'urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ',
                             os.path.join(urdf_dir, 'vehicle.urdf.xacro')]),
                    value_type=str),
            }]),
    ])
