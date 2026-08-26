from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    teleop_launch_path = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )

    motor_node = Node(
        package='motor_node',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[{
            # 조향 Profile Position (rbio 기본값)
            'steering_profile_velocity': 40000,
            'steering_profile_acceleration': 40000,
            'steering_profile_deceleration': 40000,
            'max_steering_angle_deg': 55.0,
            # cmd_vel 상한 / 타임아웃
            'max_linear_vel': 0.2,
            'max_angular_vel': 0.3,
            'cmd_vel_timeout_s': 0.5,
            # 디프 모드 (/mode 2): 전후륜을 이 각도로 돌려 고정. 0 이면 공통값 사용.
            # linear.x(또는 y) = 횡이동(+ 좌), angular.z = 제자리 회전
            'diff_steer_deg': 90.0,
            'diff_steer_front_deg': 0.0,
            'diff_steer_rear_deg': 0.0,
            'diff_max_linear_vel': 0.0,      # 0 = max_linear_vel
            'diff_max_angular_vel': 0.0,     # 0 = max_angular_vel
            'diff_linear_x_is_lateral': True,
            # 4WS 휠 오도메트리. odom->base_link TF 는 fast_lio 의 tf_2d.py 가
            # 내므로 여기서는 /odom 토픽만 낸다. 단독 테스트 시 True 로.
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_odom_tf': False,
            'odom_wheel_deadband_mps': 0.005,
        }]
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_path)
    )

    return LaunchDescription([
        motor_node,
        teleop_launch
    ])
