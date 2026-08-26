"""하부 모터노드만 띄운다 (조이스틱 없음).

bringup 의 bringup.launch.py 가 motor:=true 로 포함한다. 조이스틱 수동주행까지
필요하면 이 패키지의 bringup.launch.py 를 쓴다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
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
        ),
    ])
