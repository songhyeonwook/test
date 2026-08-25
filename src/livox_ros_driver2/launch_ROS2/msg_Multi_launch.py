import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = os.path.abspath(os.path.join(cur_path, '../config'))
user_config_path = os.path.join(cur_config_path, 'multi_MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )
    
    # IMU는 별도 장비(myahrs)가 아니라 상단 MID-360 내장 IMU를 쓴다.
    # multi_topic=1 이므로 드라이버가 라이다마다 /livox/imu_<ip> 로 따로 낸다.
    #   /livox/imu_192_168_2_102   <- 상단(1번), FAST-LIO 입력
    #   /livox/imu_192_168_3_144
    tf_livox_to_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_livox_front',
        arguments=['0.697', '0.35', '-0.18', '0', '0', '0.7071068', '0.7071068', 'livox_frame', 'livox_front']
    )

    tf_livox_to_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_livox_rear',
        arguments=['-0.6', '-0.34', '-0.18', '0', '0', '-0.7071068', '0.7071068', 'livox_frame', 'livox_rear']
    )

    # imu_link 는 이제 상단 MID-360 내부 IMU 위치를 가리킨다. 아래 값은
    # 예전 외장 IMU 시절의 identity 그대로이므로, 상단 라이다 장착 자세가
    # 확정되면 그에 맞춰 갱신해야 한다. (FAST-LIO 자체는 이 TF가 아니라
    # config 의 extrinsic_T/extrinsic_R 을 쓰므로 표시용이다.)
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'imu_link']
    )
    tf_body_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'livox_frame']
    )

    return LaunchDescription([
        livox_driver,
        tf_livox_to_front, 
        tf_livox_to_rear ,
        tf_base_to_imu,
        tf_body_to_livox
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
