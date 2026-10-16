#!/usr/bin/env python3
"""HY 전후진 방향 연동 근거리 PointCloud 안전 정지 게이트.

세 Livox가 base_link로 병합된 sliced cloud를 직접 검사한다. 전진 명령에는
전방, 후진 명령에는 후방 영역만 적용하므로 반대편의 벽 때문에 불필요하게
멈추지 않는다. cloud 수신 시각을 사용해 FAST-LIO timestamp 지연과 분리한다.
"""

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class DirectionalCollisionGateHY(Node):
    def __init__(self):
        super().__init__('directional_collision_gate_HY')
        defaults = {
            'input_cloud_topic': '/livox_merge/merged_pointcloud_sliced',
            'cmd_vel_in_topic': '/cmd_vel_collision_monitor_in',
            'cmd_vel_out_topic': '/cmd_vel_collision_monitor_out',
            'base_frame': 'base_link',
            'front_x_min': 0.75,
            'front_x_max': 1.40,
            'rear_x_min': -1.40,
            'rear_x_max': -0.75,
            'zone_y_abs': 0.65,
            'min_height': 0.15,
            'max_height': 2.0,
            'max_points': 3,
            'clear_frames': 2,
            'direction_deadband': 0.01,
            'cloud_timeout': 0.8,
            # 출장 플랫폼에서 merged stop 점이 어느 라이다에서 들어오는지
            # 확인하기 위한 진단. HY 설정은 기본 false라 영향을 받지 않는다.
            'source_diagnostics_enabled': False,
            'source_diagnostic_names': ['front', 'rear', 'top'],
            'source_diagnostic_topics': [
                '/livox/lidar_192_168_2_102',
                '/livox/lidar_192_168_3_144',
                '/livox/lidar_192_168_1_135',
            ],
            'source_diagnostic_period': 1.0,
            # merged_pointcloud_sliced와 같은 자차 제거 범위.
            'source_crop_half_x': 0.90,
            'source_crop_half_y': 0.55,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        self.max_points = int(self.max_points)
        self.clear_frames = int(self.clear_frames)

        self.front_blocked = False
        self.rear_blocked = False
        self.front_clear_count = 0
        self.rear_clear_count = 0
        self.front_points = 0
        self.rear_points = 0
        self.last_cloud_received_ns = 0
        self.last_stop_reason = ''
        self.source_diagnostic_last_ns = {}
        self.source_subscriptions = []

        self.publisher = self.create_publisher(
            Twist, str(self.cmd_vel_out_topic), 10)
        self.status_publisher = self.create_publisher(
            String, '/directional_collision_gate_HY/status', 10)
        self.create_subscription(
            PointCloud2, str(self.input_cloud_topic), self.on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            Twist, str(self.cmd_vel_in_topic), self.on_cmd_vel, 20)
        if bool(self.source_diagnostics_enabled):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self, spin_thread=False)
            names = list(self.source_diagnostic_names)
            topics = list(self.source_diagnostic_topics)
            if len(names) != len(topics):
                self.get_logger().error(
                    'Source diagnostics disabled: names/topics length mismatch')
            else:
                for name, topic in zip(names, topics):
                    sub = self.create_subscription(
                        PointCloud2, str(topic),
                        lambda msg, source=str(name):
                            self.on_source_cloud(msg, source),
                        qos_profile_sensor_data)
                    self.source_subscriptions.append(sub)
                self.get_logger().info(
                    'Per-LiDAR stop-zone diagnostics enabled: '
                    + ', '.join(f'{n}={t}' for n, t in zip(names, topics)))
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            'HY directional collision gate started: forward=front, reverse=rear')

    @staticmethod
    def read_xyz(msg):
        structured = point_cloud2.read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if structured.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.column_stack((
            structured['x'], structured['y'], structured['z'])).astype(
                np.float64, copy=False)

    def update_blocked(self, occupied, direction):
        if direction == 'front':
            if occupied:
                self.front_blocked = True
                self.front_clear_count = 0
            elif self.front_blocked:
                self.front_clear_count += 1
                if self.front_clear_count >= self.clear_frames:
                    self.front_blocked = False
                    self.front_clear_count = 0
        else:
            if occupied:
                self.rear_blocked = True
                self.rear_clear_count = 0
            elif self.rear_blocked:
                self.rear_clear_count += 1
                if self.rear_clear_count >= self.clear_frames:
                    self.rear_blocked = False
                    self.rear_clear_count = 0

    @staticmethod
    def transform_points(points, transform):
        q = transform.transform.rotation
        t = transform.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),
             2.0 * (x*z + y*w)],
            [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z),
             2.0 * (y*z - x*w)],
            [2.0 * (x*z - y*w), 2.0 * (y*z + x*w),
             1.0 - 2.0 * (x*x + y*y)],
        ], dtype=np.float64)
        return points @ rotation.T + np.array([t.x, t.y, t.z])

    def zone_masks(self, points):
        height = (
            (points[:, 2] >= float(self.min_height))
            & (points[:, 2] <= float(self.max_height)))
        lateral = np.abs(points[:, 1]) <= float(self.zone_y_abs)
        front = (height & lateral
                 & (points[:, 0] >= float(self.front_x_min))
                 & (points[:, 0] <= float(self.front_x_max)))
        rear = (height & lateral
                & (points[:, 0] >= float(self.rear_x_min))
                & (points[:, 0] <= float(self.rear_x_max)))
        return front, rear

    @staticmethod
    def format_bounds(points):
        if len(points) == 0:
            return 'none'
        lo = np.min(points, axis=0)
        hi = np.max(points, axis=0)
        return (f'n={len(points)} x={lo[0]:.2f}..{hi[0]:.2f} '
                f'y={lo[1]:.2f}..{hi[1]:.2f} '
                f'z={lo[2]:.2f}..{hi[2]:.2f}')

    def on_source_cloud(self, msg, source):
        """각 raw LiDAR를 base_link로 변환해 merged slice와 동일하게 검사."""
        try:
            points = self.read_xyz(msg)
            transform = self.tf_buffer.lookup_transform(
                str(self.base_frame), msg.header.frame_id, Time())
            points = self.transform_points(points, transform)
        except Exception as exc:  # TF 미준비/PointCloud 필드 오류는 진단만 건너뜀
            now_ns = self.get_clock().now().nanoseconds
            last_ns = self.source_diagnostic_last_ns.get(source, 0)
            if now_ns - last_ns >= int(float(self.source_diagnostic_period) * 1e9):
                self.source_diagnostic_last_ns[source] = now_ns
                self.get_logger().warning(
                    f'LIDAR_DIAG {source}: unable to inspect {msg.header.frame_id}: {exc}')
            return

        # merged_pointcloud_sliced의 높이/자차 crop을 그대로 재현한다.
        self_crop = (
            (np.abs(points[:, 0]) <= float(self.source_crop_half_x))
            & (np.abs(points[:, 1]) <= float(self.source_crop_half_y)))
        points = points[~self_crop]
        front_mask, rear_mask = self.zone_masks(points)
        front = points[front_mask]
        rear = points[rear_mask]
        if len(front) <= self.max_points and len(rear) <= self.max_points:
            return

        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.source_diagnostic_last_ns.get(source, 0)
        if now_ns - last_ns < int(float(self.source_diagnostic_period) * 1e9):
            return
        self.source_diagnostic_last_ns[source] = now_ns
        lateral_counts = ','.join(
            f'|y|<={limit:.2f}:{int(np.count_nonzero(np.abs(front[:, 1]) <= limit))}'
            for limit in (0.30, 0.40, 0.48, 0.55, 0.60))
        self.get_logger().warning(
            f'LIDAR_DIAG {source}: front[{self.format_bounds(front)}] '
            f'rear[{self.format_bounds(rear)}] front_lateral[{lateral_counts}]')

    def on_cloud(self, msg):
        if msg.header.frame_id != str(self.base_frame):
            self.get_logger().error(
                f'Ignoring cloud frame {msg.header.frame_id}; expected {self.base_frame}')
            return
        try:
            points = self.read_xyz(msg)
        except (AssertionError, KeyError, ValueError) as exc:
            self.get_logger().warning(f'Invalid PointCloud2 xyz fields: {exc}')
            return

        front_mask, rear_mask = self.zone_masks(points)
        self.front_points = int(np.count_nonzero(front_mask))
        self.rear_points = int(np.count_nonzero(rear_mask))
        self.update_blocked(self.front_points > self.max_points, 'front')
        self.update_blocked(self.rear_points > self.max_points, 'rear')
        self.last_cloud_received_ns = self.get_clock().now().nanoseconds

    def on_cmd_vel(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        moving = (
            abs(msg.linear.x) > float(self.direction_deadband)
            or abs(msg.linear.y) > float(self.direction_deadband))
        cloud_fresh = (
            self.last_cloud_received_ns > 0
            and (now_ns - self.last_cloud_received_ns) / 1e9
            <= float(self.cloud_timeout))

        reason = ''
        if moving and not cloud_fresh:
            reason = 'CLOUD_STALE'
        elif msg.linear.x > float(self.direction_deadband) and self.front_blocked:
            reason = 'FRONT'
        elif msg.linear.x < -float(self.direction_deadband) and self.rear_blocked:
            reason = 'REAR'

        if reason:
            output = Twist()
            if reason != self.last_stop_reason:
                self.get_logger().warning(
                    f'STOP {reason}: front_points={self.front_points}, '
                    f'rear_points={self.rear_points}')
        else:
            output = msg
            if self.last_stop_reason and moving:
                self.get_logger().info(
                    f'RESUME from {self.last_stop_reason}: '
                    f'front_points={self.front_points}, rear_points={self.rear_points}')
        self.last_stop_reason = reason
        self.publisher.publish(output)

    def publish_status(self):
        age = -1.0
        if self.last_cloud_received_ns > 0:
            age = (self.get_clock().now().nanoseconds
                   - self.last_cloud_received_ns) / 1e9
        msg = String()
        msg.data = (
            f'front_blocked={self.front_blocked} front_points={self.front_points} '
            f'rear_blocked={self.rear_blocked} rear_points={self.rear_points} '
            f'cloud_age={age:.3f}s stop={self.last_stop_reason or "NONE"}')
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DirectionalCollisionGateHY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
