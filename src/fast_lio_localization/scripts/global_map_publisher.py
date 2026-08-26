#!/usr/bin/env python3

import os

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        # 기본값을 비워 둔다. 경로는 localization.launch.py 의 map 인자가 넘긴다.
        # 하드코딩된 기본값이 남아 있으면 인자를 바꿔도 조용히 옛 맵을 읽는다.
        self.declare_parameter('map_file_path', '')
        self.declare_parameter('interval', 5)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value
        
        self.global_map = None
        if not path:
            self.get_logger().error(
                'map_file_path 가 비어 있다. localization.launch.py 의 map 인자를 확인할 것.')
        elif not os.path.isfile(path):
            # open3d 는 없는 파일에 예외를 내지 않고 빈 클라우드를 준다. 그대로 두면
            # "맵을 읽었다" 고 찍힌 채 ICP 가 영원히 수렴하지 않는다. 여기서 끊는다.
            self.get_logger().error(
                f'3D prior map 이 없다: {path}  '
                '(map 인자의 이름과 navigation/map 의 .pcd 가 짝이 맞는지 확인. '
                '새로 넣은 파일이면 colcon build 를 한 번 해야 share 에 링크가 생긴다)')
        else:
            try:
                cloud = o3d.io.read_point_cloud(path)
                if len(cloud.points) == 0:
                    self.get_logger().error(f'3D prior map 이 비어 있다(점 0개): {path}')
                else:
                    self.global_map = cloud
                    self.get_logger().info(
                        f'Loaded map from: {path} ({len(cloud.points)} points)')
            except Exception as e:
                self.get_logger().error(f'Failed to load PCD: {e}')

        self.pub_map = self.create_publisher(PointCloud2, '/global_map', 1)
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Interval for publishing map: {interval} seconds')
        self.get_logger().info('Map Publisher Node Initialized')

    def publish_map(self):
        if self.global_map is None:
            self.get_logger().warn('Global map is not loaded; skipping publish')
            return
        points = np.asarray(self.global_map.points)
        if points.size == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        try:
            cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        except AttributeError:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud = point_cloud2.create_cloud(header, fields, points.tolist())
        self.pub_map.publish(cloud)
        # self.get_logger().info('Published global map')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()        
