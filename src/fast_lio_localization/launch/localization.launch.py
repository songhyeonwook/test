import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio_localization')
    default_config_path = os.path.join(package_path, 'config')
    # rviz 설정은 이 패키지 안에, 2D 점유맵은 bringup 이 들고 있다.
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio_localization.rviz')
    default_map_yaml_path = os.path.join(
        get_package_share_directory('bringup'), 'maps', 'scans_new.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    map_yaml = LaunchConfiguration('map_yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='YAML config file path'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='mid360.yaml',
        description='Config file name'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Use RViz to monitor results'
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml_path,
        description='2D occupancy map yaml path'
    )

    # FAST-LIO localization
    fast_lio_node = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    global_localization_node = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    transform_fusion_node = Node(
        package='fast_lio_localization',
        executable='transform_fusion.py',
        name='transform_fusion',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    global_map_publisher_node = Node(
        package='fast_lio_localization',
        executable='global_map_publisher.py',
        name='global_map_publisher',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 2D TF bridge: map -> odom -> base_link
    tf_2d_bridge_node = Node(
        package='fast_lio_localization',
        executable='tf_2d.py',
        name='tf_2d_bridge',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'parent_3d': 'map'},
            {'odom_2d': 'odom'},
            {'base_2d': 'base_link'},
            {'caminit': 'camera_init'},
            # FAST-LIO 의 'body' 는 IMU 프레임이라 그대로 평면화하면 base_link 가
            # 마스트 위(1.4m, 측방 0.56m)에 생긴다. nav2 footprint 가 base_link
            # 기준 대칭이므로 차량 기준점을 평면화해야 한다.
            {'body': 'base_footprint'},
            {'rate_hz': 50.0}
        ],
        output='screen'
    )

    # ROS2 map_server (2D map frame = prior_map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'frame_id': 'map'
            }
        ]
    )

    # lifecycle manager for map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use),
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(global_map_publisher_node)
    ld.add_action(tf_2d_bridge_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld
