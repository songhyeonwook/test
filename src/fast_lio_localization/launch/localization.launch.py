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
    # rviz 설정은 이 패키지 안에, 맵은 navigation 패키지가 들고 있다.
    # 2D(점유맵)와 3D(prior map)는 같은 주행에서 나온 짝이어야 하므로 한 곳에 둔다.
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio_localization.rviz')
    default_map_yaml_path = os.path.join(
        get_package_share_directory('navigation'), 'map', 'test.yaml')

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

    # FAST-LIO 내부 프레임(camera_init)을 TF 에 올릴지. 정식 트리는
    # map->odom->base_link 뿐이고, RViz 에서 /cloud_registered 같은
    # camera_init 프레임 토픽을 map 위에 겹쳐 볼 때만 켠다.
    debug_tf = LaunchConfiguration('debug_tf')
    declare_debug_tf_cmd = DeclareLaunchArgument(
        'debug_tf',
        default_value='false',
        description='Publish debug TF map->camera_init (RViz inspection only)'
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
            {'use_sim_time': use_sim_time},
            {'publish_debug_tf': debug_tf}
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

    # 2D TF bridge: /Odometry + /map_to_odom 토픽 -> map -> odom -> base_link
    # body(IMU) -> 차량 기준점 extrinsic(ref_from_body_*)은 mid360.yaml 에 있다.
    tf_2d_bridge_node = Node(
        package='fast_lio_localization',
        executable='tf_2d.py',
        name='tf_2d_bridge',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
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
    ld.add_action(declare_debug_tf_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(global_map_publisher_node)
    ld.add_action(tf_2d_bridge_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld
