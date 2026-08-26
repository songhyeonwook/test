"""URDF 를 읽어 TF 를 내보낸다.

두 개의 트리를 따로 발행한다.

  vehicle       base_link -> base_footprint -> 센서들      (사용자가 보는 모델)
  fastlio_ref   body -> fastlio_ref                        (tf_2d.py 의 평면화 대상)

FAST-LIO 가 camera_init->body 를, tf_2d.py 가 map->odom->base_link 를 내므로
전체는 이렇게 이어진다.

  map ─┬─ camera_init ── body ── fastlio_ref
       └─ odom ── base_link ── base_footprint ── livox_*

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

    def rsp(name, xacro_file, topic):
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ', os.path.join(urdf_dir, xacro_file)]),
                    value_type=str),
            }],
            remappings=[('robot_description', topic)])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # RViz 의 RobotModel 이 보는 쪽
        rsp('robot_state_publisher', 'vehicle.urdf.xacro', 'robot_description'),
        # FAST-LIO 배관. 모델이 아니라 프레임 하나뿐이라 토픽을 따로 쓴다.
        rsp('fastlio_ref_state_publisher', 'fastlio_ref.urdf.xacro',
            'fastlio_ref_description'),
    ])
