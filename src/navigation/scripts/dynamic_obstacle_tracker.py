#!/usr/bin/env python3
"""HY LiDAR 기반 정적/동적 장애물 추적기.

로봇 자체 이동을 제거하기 위해 cloud를 odom 좌표계에서 군집/추적한다.
처음 정지해 있던 군집도 이후 연속 이동이 확인되면 DYNAMIC_LATCHED로
승격한다. 한 번 승격된 track은 다시 멈춰도 동적 상태를 유지하며, 전후방
안전영역 안에 있는 동안 Collision Monitor용 synthetic cloud를 발행한다.

이 노드는 안전의 유일한 계층이 아니다. 분류 실패에 대비한 raw cloud 기반
근거리 Collision Monitor를 반드시 함께 사용한다.
"""

from collections import deque
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
def apply_transform(points, rotation, translation):
    return points @ rotation.T + translation


class Track:
    def __init__(self, track_id, position, extent, point_count, stamp_ns, now_ns):
        self.track_id = track_id
        self.position = position.copy()
        self.anchor = position.copy()
        self.extent = extent.copy()
        self.point_count = point_count
        self.velocity = np.zeros(2, dtype=np.float64)
        self.motion_direction = np.zeros(2, dtype=np.float64)
        self.moving_count = 0
        self.motion_candidate_since_ns = 0
        self.dynamic = False
        self.provisional_was_stopping = False
        self.last_stamp_ns = stamp_ns
        self.last_seen_ns = now_ns
        self.was_stopping = False
        # 동적 객체의 cluster ID가 끊긴 경우, 마지막 위치 주변이 실제로
        # 비었는지 연속 확인하기 위한 시작 시각이다.
        self.lost_clear_since_ns = 0
        self.lost_evidence_holding = False


class DynamicObstacleTrackerHY(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_tracker_HY')

        defaults = {
            # 연구실 HY는 검증된 기존 동작을 유지한다. 출장 전용 entrypoint가
            # nav2_params.yaml에서만 true로 켜 CAN/heartbeat 보완을 활성화한다.
            'field_hardening_enabled': False,
            'input_topic': '/livox_merge/merged_pointcloud_sliced',
            'output_topic': '/dynamic_obstacles_HY',
            'tracking_frame': 'odom',
            'base_frame': 'base_link',
            # 출장 파라미터에서는 라이다와 동일 시간축인 FAST-LIO /Odometry를
            # 지정한다. 기본값은 독립 실행 시 하위호환을 위해 유지한다.
            'ego_odom_topic': '/odom',
            'ego_odom_history_seconds': 5.0,
            'ego_odom_tolerance': 0.15,
            # motor_node는 TPDO가 stale이면 pose covariance를 1e6으로
            # 올린다. 그 구간의 정지된 wheel odom으로 상대운동을 보상하면
            # 고정 장애물이 움직인 것처럼 보여 동적 오검출이 발생한다.
            'ego_odom_max_covariance': 1000.0,
            # map localization을 ego-motion 기준으로 쓸 때 ICP가 비현실적으로
            # 점프하면 모든 정적 군집이 움직인 것처럼 보인다. 이 한계를 넘는
            # pose 변화에서는 track/cache를 전부 초기화한다.
            'ego_pose_max_linear_speed': 0.80,
            'ego_pose_max_angular_speed': 0.80,
            # CAN/TPDO가 복구된 직후에는 frozen pose와 현재 pose 사이의
            # 불연속이 남을 수 있다. 정상 odom이 이 시간 동안 연속으로
            # 들어온 뒤에만 새 정적 track의 동적 승격을 허용한다.
            'ego_odom_recovery_seconds': 0.50,
            'roi_x_min': -3.0,
            'roi_x_max': 3.0,
            'roi_y_abs': 2.0,
            'voxel_size': 0.10,
            'cluster_tolerance': 0.24,
            'min_cluster_voxels': 3,
            'max_cluster_extent': 1.50,
            'association_distance': 0.40,
            'merge_ambiguity_distance': 0.45,
            'max_extent_change': 0.35,
            'max_point_count_ratio': 3.0,
            'dynamic_speed_threshold': 0.15,
            'dynamic_min_displacement': 0.08,
            'dynamic_confirm_frames': 2,
            # speed/displacement 조건을 처음 넘은 즉시 latch하지 않는다.
            # CAN stale 판정 직전의 짧은 pose 불연속은 이 시간 안에 odom
            # invalid로 드러나므로, 연속 이동이 유지될 때만 동적으로 확정한다.
            'dynamic_confirmation_time': 1.0,
            # Collision Monitor의 source timeout보다 빠르게 현재 정지 cloud를
            # 반복 발행한다. wheel odom이 잠시 끊겨도 이미 확인된 장애물로
            # 인한 정지가 중간에 풀려 로봇이 찔끔 움직이지 않게 한다.
            'dynamic_cloud_heartbeat_period': 0.05,
            'min_motion_direction_cosine': 0.20,
            'velocity_filter_alpha': 0.50,
            'static_anchor_alpha': 0.05,
            'static_track_timeout': 0.8,
            'dynamic_lost_hold_time': 2.0,
            # ID 변경/일시 가림을 단순 timeout으로 해제하지 않는다. 마지막
            # 동적 track 주변의 실제 군집이 사라진 상태가 연속 확인돼야 한다.
            'lost_track_evidence_radius': 0.60,
            'lost_release_clear_time': 1.00,
            # 동적으로 확정된 객체는 진행 방향과 무관하게 차체 주변에서
            # 정지시킨다. 세 라이다 중 어느 센서가 본 점인지와도 무관하다.
            'surround_stop_zone_x_abs': 2.00,
            'surround_stop_zone_y_abs': 1.00,
            'synthetic_points_per_track': 6,
            'cloud_queue_size': 30,
            'cloud_wait_timeout': 0.60,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.min_cluster_voxels = int(self.min_cluster_voxels)
        self.dynamic_confirm_frames = int(self.dynamic_confirm_frames)
        self.synthetic_points_per_track = int(self.synthetic_points_per_track)
        self.cloud_queue_size = int(self.cloud_queue_size)

        self.publisher = self.create_publisher(
            PointCloud2, str(self.output_topic), 10)
        self.status_publisher = self.create_publisher(
            String, '/dynamic_obstacle_tracker_HY/status', 10)
        self.create_subscription(
            PointCloud2, str(self.input_topic), self.on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.ego_odom_topic), self.on_ego_odom, 50)

        self.tracks = {}
        self.next_track_id = 1
        self.ego_odom_history = deque()
        self.pending_clouds = deque()
        self.processing_clouds = False
        self.last_tf_warning_ns = 0
        self.last_queue_warning_ns = 0
        self.last_invalid_odom_warning_ns = 0
        self.ego_odom_valid_since_ns = 0
        self.last_ego_odom_valid = False
        self.last_status_ns = 0
        self.cached_dynamic_output_points = []
        self.cached_provisional_output_points = []
        self.cloud_timer = self.create_timer(0.02, self.process_pending_clouds)
        self.dynamic_cloud_timer = None
        if bool(self.field_hardening_enabled):
            self.dynamic_cloud_timer = self.create_timer(
                float(self.dynamic_cloud_heartbeat_period),
                self.publish_dynamic_heartbeat)
        self.get_logger().info(
            'HY dynamic tracker started: STATIC objects can be promoted to '
            'DYNAMIC_LATCHED after later motion')

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def pose_matrix(x, y, yaw):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.array([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        translation = np.array([x, y, 0.0], dtype=np.float64)
        return rotation, translation

    def on_ego_odom(self, msg):
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        covariance_values = (
            float(covariance[0]),
            float(covariance[7]),
            float(covariance[35]))
        odom_valid = (
            not bool(self.field_hardening_enabled)
            or (all(math.isfinite(value) for value in covariance_values)
                and max(covariance_values)
                < float(self.ego_odom_max_covariance)))
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = self.yaw_from_quaternion(pose.orientation)
        pose_discontinuity = False
        linear_speed = 0.0
        angular_speed = 0.0
        if (bool(self.field_hardening_enabled)
                and odom_valid and self.ego_odom_history):
            previous = self.ego_odom_history[-1]
            dt = (stamp_ns - previous[0]) / 1e9
            if previous[4] and 0.02 <= dt <= 1.0:
                linear_speed = math.hypot(
                    x - previous[1], y - previous[2]) / dt
                yaw_delta = math.atan2(
                    math.sin(yaw - previous[3]),
                    math.cos(yaw - previous[3]))
                angular_speed = abs(yaw_delta) / dt
                pose_discontinuity = (
                    linear_speed > float(self.ego_pose_max_linear_speed)
                    or angular_speed > float(
                        self.ego_pose_max_angular_speed))

        if pose_discontinuity:
            # 좌표계가 순간 이동한 전후의 centroid는 비교할 수 없다. 확정된
            # dynamic까지 포함해 모두 폐기하지 않으면 존재하지 않는 장애물로
            # heartbeat 정지가 영구 유지될 수 있다.
            self.tracks.clear()
            self.ego_odom_history.clear()
            self.pending_clouds.clear()
            self.cached_dynamic_output_points = []
            self.cached_provisional_output_points = []
            self.last_ego_odom_valid = False
            self.ego_odom_valid_since_ns = 0
            self.get_logger().warning(
                'Ego pose discontinuity detected; clearing all dynamic '
                f'tracks (linear={linear_speed:.2f}m/s, '
                f'angular={angular_speed:.2f}rad/s)')
        sample = (
            stamp_ns,
            x,
            y,
            yaw,
            odom_valid)
        self.ego_odom_history.append(sample)

        if not odom_valid:
            # frozen pose를 기준으로 만들어진 정적 track은 odom 복구 후의
            # 현재 pose와 비교할 수 없다. 카운터만 지우면 오래된 anchor와의
            # 차이가 다시 이동량으로 잡히므로 정적 track 자체를 폐기한다.
            # 이미 동적으로 확정된 track은 보존하여 안전 정지가 풀리지 않게 한다.
            if self.last_ego_odom_valid:
                self.tracks = {
                    track_id: track for track_id, track in self.tracks.items()
                    if track.dynamic}
            self.last_ego_odom_valid = False
            self.ego_odom_valid_since_ns = 0
            # 임시 후보는 odom 검증 실패 즉시 폐기한다. 이미 확정된 동적
            # 장애물의 cache만 유지하여 안전 정지는 보수적으로 지속한다.
            self.cached_provisional_output_points = []
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_invalid_odom_warning_ns > 2_000_000_000:
                self.get_logger().warning(
                    'Ignoring ego odometry marked invalid; STATIC tracks '
                    'cannot be promoted until odometry recovers')
                self.last_invalid_odom_warning_ns = now_ns
        else:
            if not self.last_ego_odom_valid:
                self.ego_odom_valid_since_ns = stamp_ns
            self.last_ego_odom_valid = True
        oldest_ns = stamp_ns - int(
            float(self.ego_odom_history_seconds) * 1_000_000_000)
        while (len(self.ego_odom_history) > 2
               and self.ego_odom_history[1][0] < oldest_ns):
            self.ego_odom_history.popleft()
        self.process_pending_clouds()

    def interpolate_ego_pose(self, stamp_ns):
        if not self.ego_odom_history:
            return None
        history = self.ego_odom_history
        tolerance_ns = int(float(self.ego_odom_tolerance) * 1_000_000_000)
        recovery_ns = int(
            float(self.ego_odom_recovery_seconds) * 1_000_000_000)
        effective_stamp_ns = history[-1][0] if stamp_ns <= 0 else stamp_ns
        # 복구 직후 cloud를 처리하면 CAN 단절 전후 pose 불연속을 장애물
        # 이동으로 오인한다. scan 시각까지 정상 구간이 충분히 누적돼야 한다.
        if (bool(self.field_hardening_enabled)
                and (self.ego_odom_valid_since_ns <= 0
                     or effective_stamp_ns
                     < self.ego_odom_valid_since_ns + recovery_ns)):
            return None
        if stamp_ns <= 0:
            return history[-1][1:4] if history[-1][4] else None
        if stamp_ns <= history[0][0]:
            return (history[0][1:4]
                    if history[0][4]
                    and history[0][0] - stamp_ns <= tolerance_ns else None)
        if stamp_ns >= history[-1][0]:
            return (history[-1][1:4]
                    if history[-1][4]
                    and stamp_ns - history[-1][0] <= tolerance_ns else None)

        for index in range(len(history) - 1, 0, -1):
            before = history[index - 1]
            after = history[index]
            if before[0] <= stamp_ns <= after[0]:
                # stale 구간 양쪽의 정상 pose를 직선 보간하면 stale 동안에도
                # 차체가 정상 이동한 것처럼 만들어 오검출이 재발한다.
                if not before[4] or not after[4]:
                    return None
                interval = after[0] - before[0]
                if interval <= 0 or interval > 2 * tolerance_ns:
                    return None
                ratio = (stamp_ns - before[0]) / interval
                yaw_delta = math.atan2(
                    math.sin(after[3] - before[3]),
                    math.cos(after[3] - before[3]))
                return (
                    before[1] + ratio * (after[1] - before[1]),
                    before[2] + ratio * (after[2] - before[2]),
                    before[3] + ratio * yaw_delta)
        return None

    def base_to_tracking_matrix(self, stamp_ns):
        pose = self.interpolate_ego_pose(stamp_ns)
        return None if pose is None else self.pose_matrix(*pose)

    def tracking_to_current_base_matrix(self):
        if not self.ego_odom_history:
            return None
        latest = self.ego_odom_history[-1]
        if not latest[4]:
            return None
        rotation, translation = self.pose_matrix(*latest[1:4])
        inverse_rotation = rotation.T
        return inverse_rotation, -(inverse_rotation @ translation)

    def read_xyz(self, msg):
        try:
            structured = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)
            if structured.size == 0:
                return np.empty((0, 3), dtype=np.float64)
            return np.column_stack((
                structured['x'], structured['y'], structured['z'])).astype(
                    np.float64, copy=False)
        except (AssertionError, KeyError, ValueError) as exc:
            self.get_logger().warning(f'Invalid PointCloud2 xyz fields: {exc}')
            return np.empty((0, 3), dtype=np.float64)

    def cluster(self, points_base, points_odom):
        if len(points_base) == 0:
            return []
        roi = (
            (points_base[:, 0] >= float(self.roi_x_min))
            & (points_base[:, 0] <= float(self.roi_x_max))
            & (np.abs(points_base[:, 1]) <= float(self.roi_y_abs)))
        points = points_odom[roi]
        if len(points) == 0:
            return []

        cells = np.floor(points[:, :2] / float(self.voxel_size)).astype(np.int32)
        unique_cells, inverse, counts = np.unique(
            cells, axis=0, return_inverse=True, return_counts=True)
        sums = np.zeros((len(unique_cells), 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        voxel_points = sums / counts[:, None]

        cell_to_index = {tuple(cell): i for i, cell in enumerate(unique_cells)}
        neighbor_cells = int(math.ceil(
            float(self.cluster_tolerance) / float(self.voxel_size)))
        tolerance_sq = float(self.cluster_tolerance) ** 2
        unvisited = set(range(len(unique_cells)))
        clusters = []

        while unvisited:
            seed = unvisited.pop()
            component = [seed]
            queue = deque([seed])
            while queue:
                current = queue.popleft()
                cx, cy = unique_cells[current]
                for dx in range(-neighbor_cells, neighbor_cells + 1):
                    for dy in range(-neighbor_cells, neighbor_cells + 1):
                        neighbor = cell_to_index.get((cx + dx, cy + dy))
                        if neighbor is None or neighbor not in unvisited:
                            continue
                        delta = voxel_points[neighbor, :2] - voxel_points[current, :2]
                        if float(delta @ delta) <= tolerance_sq:
                            unvisited.remove(neighbor)
                            component.append(neighbor)
                            queue.append(neighbor)

            if len(component) < self.min_cluster_voxels:
                continue
            component = np.asarray(component, dtype=np.int32)
            cluster_points = voxel_points[component]
            extent = np.ptp(cluster_points[:, :2], axis=0)
            if max(extent) > float(self.max_cluster_extent):
                continue
            weights = counts[component].astype(np.float64)
            centroid = np.average(cluster_points, axis=0, weights=weights)
            clusters.append((centroid, extent, int(np.sum(weights))))
        return clusters

    def associate_and_update(self, clusters, stamp_ns, now_ns):
        # 사람이 의자 옆을 지나갈 때 두 군집이 하나로 합쳐지면 centroid가
        # 크게 움직여 정적 의자가 동적으로 보일 수 있다. 여러 기존 track과
        # 동시에 가까운 군집은 identity가 불명확하므로 승격에 사용하지 않는다.
        ambiguous_clusters = set()
        for cluster_index, (centroid, _, _) in enumerate(clusters):
            nearby = sum(
                np.linalg.norm(track.position[:2] - centroid[:2])
                <= float(self.merge_ambiguity_distance)
                for track in self.tracks.values())
            if nearby >= 2:
                ambiguous_clusters.add(cluster_index)

        assignments = []
        for track_id, track in self.tracks.items():
            for cluster_index, (centroid, extent, point_count) in enumerate(clusters):
                if cluster_index in ambiguous_clusters:
                    continue
                distance = float(np.linalg.norm(track.position[:2] - centroid[:2]))
                extent_change = float(np.linalg.norm(extent - track.extent))
                count_ratio = max(point_count, track.point_count) / max(
                    1, min(point_count, track.point_count))
                shape_ok = (
                    extent_change <= float(self.max_extent_change)
                    and count_ratio <= float(self.max_point_count_ratio))

                if distance <= float(self.association_distance) and shape_ok:
                    assignments.append((distance, track_id, cluster_index))
        assignments.sort()

        used_tracks = set()
        # 합쳐진 군집으로 새 track까지 만들면 분리 순간 또 가짜 속도가 생긴다.
        used_clusters = set(ambiguous_clusters)
        for _, track_id, cluster_index in assignments:
            if track_id in used_tracks or cluster_index in used_clusters:
                continue
            used_tracks.add(track_id)
            used_clusters.add(cluster_index)
            centroid, extent, point_count = clusters[cluster_index]
            self.update_track(
                self.tracks[track_id], centroid, extent, point_count,
                stamp_ns, now_ns)

        for cluster_index, (centroid, extent, point_count) in enumerate(clusters):
            if cluster_index in used_clusters:
                continue
            track = Track(
                self.next_track_id, centroid, extent, point_count,
                stamp_ns, now_ns)
            self.tracks[track.track_id] = track
            self.next_track_id += 1

        stale_ids = []
        for track_id, track in self.tracks.items():
            age = (now_ns - track.last_seen_ns) / 1e9
            timeout = (
                float(self.dynamic_lost_hold_time) if track.dynamic
                else float(self.static_track_timeout))
            if age <= timeout:
                continue

            safe_release = track.dynamic and track.was_stopping
            if safe_release:
                evidence_nearby = any(
                    float(np.linalg.norm(
                        centroid[:2] - track.position[:2]))
                    <= float(self.lost_track_evidence_radius)
                    for centroid, _, _ in clusters)
                if evidence_nearby:
                    track.lost_clear_since_ns = 0
                    if not track.lost_evidence_holding:
                        self.get_logger().warning(
                            f'Dynamic track {track_id} identity lost, but obstacle '
                            'evidence remains nearby; holding stop')
                        track.lost_evidence_holding = True
                    continue

                if track.lost_clear_since_ns == 0:
                    track.lost_clear_since_ns = now_ns
                    track.lost_evidence_holding = False
                    self.get_logger().info(
                        f'Dynamic track {track_id} lost; confirming clear area '
                        'before release')
                    continue
                clear_time = (
                    now_ns - track.lost_clear_since_ns) / 1e9
                if clear_time < float(self.lost_release_clear_time):
                    continue

                self.get_logger().warning(
                    f'Dynamic track {track_id} released only after nearby area '
                    f'was clear for {clear_time:.1f}s. Raw emergency monitor remains active.')
            elif track.dynamic and track.was_stopping:
                self.get_logger().warning(
                    f'Dynamic track {track_id} lost inside stop zone; '
                    f'releasing after {age:.1f}s hold. Raw emergency monitor remains active.')
            stale_ids.append(track_id)
        for track_id in stale_ids:
            del self.tracks[track_id]

    def update_track(self, track, centroid, extent, point_count, stamp_ns, now_ns):
        dt = (stamp_ns - track.last_stamp_ns) / 1e9
        if 0.02 <= dt <= 1.0:
            step = centroid[:2] - track.position[:2]
            instantaneous = step / dt
            alpha = float(self.velocity_filter_alpha)
            track.velocity = alpha * instantaneous + (1.0 - alpha) * track.velocity
            speed = float(np.linalg.norm(track.velocity))
            displacement = float(np.linalg.norm(centroid[:2] - track.anchor[:2]))
            moving = (
                speed >= float(self.dynamic_speed_threshold)
                and displacement >= float(self.dynamic_min_displacement))
            if moving:
                step_norm = float(np.linalg.norm(step))
                direction = step / step_norm if step_norm > 1e-6 else None
                previous_norm = float(np.linalg.norm(track.motion_direction))
                consistent = (
                    direction is not None
                    and (previous_norm == 0.0 or float(np.dot(
                        direction, track.motion_direction
                    )) >= float(self.min_motion_direction_cosine)))
                if consistent:
                    if track.moving_count == 0:
                        track.motion_candidate_since_ns = stamp_ns
                    track.moving_count += 1
                else:
                    # 방향이 튄 첫 프레임은 새 움직임 후보로만 취급한다.
                    track.moving_count = 1 if direction is not None else 0
                    track.motion_candidate_since_ns = (
                        stamp_ns if direction is not None else 0)
                if direction is not None:
                    track.motion_direction = direction
            else:
                if bool(self.field_hardening_enabled):
                    # 출장 플랫폼은 실제 연속 이동만 확정한다. 한 프레임이라도
                    # 조건 아래면 후보를 초기화해 CAN/점군 흔들림 누적을 막는다.
                    track.moving_count = 0
                    track.motion_candidate_since_ns = 0
                    if not track.dynamic:
                        track.motion_direction[:] = 0.0
                else:
                    # 검증된 HY 기존 동작: 한 프레임 누락은 감산하여 허용한다.
                    track.moving_count = max(0, track.moving_count - 1)
                    if track.moving_count == 0 and not track.dynamic:
                        track.motion_direction[:] = 0.0

            candidate_duration = (
                (stamp_ns - track.motion_candidate_since_ns) / 1e9
                if track.motion_candidate_since_ns > 0 else 0.0)
            confirmation_time = (
                float(self.dynamic_confirmation_time)
                if bool(self.field_hardening_enabled) else 0.0)
            if (not track.dynamic
                    and track.moving_count >= self.dynamic_confirm_frames
                    and candidate_duration
                    >= confirmation_time):
                track.dynamic = True
                self.get_logger().warning(
                    f'Track {track.track_id}: STATIC -> DYNAMIC_LATCHED '
                    f'(speed={speed:.2f}m/s, displacement={displacement:.2f}m, '
                    f'confirmed={candidate_duration:.2f}s)')

            if not track.dynamic and not moving:
                anchor_alpha = float(self.static_anchor_alpha)
                track.anchor = (
                    (1.0 - anchor_alpha) * track.anchor
                    + anchor_alpha * centroid)

        track.position = centroid.copy()
        track.extent = extent.copy()
        track.point_count = point_count
        track.last_stamp_ns = stamp_ns
        track.last_seen_ns = now_ns
        track.lost_clear_since_ns = 0
        track.lost_evidence_holding = False

    def is_in_stop_zone(self, point_base):
        """차체 주변 360도 동적 안전영역 안인지 확인한다."""
        return (
            abs(point_base[0]) <= float(self.surround_stop_zone_x_abs)
            and abs(point_base[1]) <= float(self.surround_stop_zone_y_abs))

    def publish_dynamic_cloud(self, now_msg, odom_to_base):
        rotation, translation = odom_to_base
        output_points = []
        dynamic_output_points = []
        provisional_output_points = []
        stopping_tracks = []
        offsets = [(0.0, 0.0), (0.025, 0.0), (-0.025, 0.0),
                   (0.0, 0.025), (0.0, -0.025), (0.018, 0.018)]

        for track in self.tracks.values():
            provisional = (
                bool(self.field_hardening_enabled)
                and not track.dynamic
                and track.moving_count >= self.dynamic_confirm_frames
                and track.motion_candidate_since_ns > 0)
            if not track.dynamic and not provisional:
                if track.provisional_was_stopping:
                    self.get_logger().info(
                        f'Motion candidate track {track.track_id} cleared; '
                        'provisional stop released')
                    track.provisional_was_stopping = False
                continue
            position_base = apply_transform(
                track.position.reshape(1, 3), rotation, translation)[0]
            stopping = self.is_in_stop_zone(position_base)
            if track.dynamic:
                track.provisional_was_stopping = False
                if stopping and not track.was_stopping:
                    self.get_logger().warning(
                        f'Dynamic track {track.track_id} entered stop zone')
                elif not stopping and track.was_stopping:
                    self.get_logger().info(
                        f'Dynamic track {track.track_id} left stop zone; '
                        'motion may resume')
                track.was_stopping = stopping
            else:
                if stopping and not track.provisional_was_stopping:
                    self.get_logger().warning(
                        f'Motion candidate track {track.track_id} entered '
                        'provisional stop zone')
                elif not stopping and track.provisional_was_stopping:
                    self.get_logger().info(
                        f'Motion candidate track {track.track_id} left '
                        'provisional stop zone')
                track.provisional_was_stopping = stopping
            if not stopping:
                continue
            stopping_tracks.append(track.track_id)
            for index in range(self.synthetic_points_per_track):
                dx, dy = offsets[index % len(offsets)]
                point = (
                    float(position_base[0] + dx),
                    float(position_base[1] + dy),
                    float(max(0.30, position_base[2])))
                output_points.append(point)
                if track.dynamic:
                    dynamic_output_points.append(point)
                else:
                    provisional_output_points.append(point)

        # process_cloud가 wheel odom 단절로 잠시 멈춰도 heartbeat가 같은 정지
        # 근거를 최신 timestamp로 계속 내보낼 수 있도록 base 좌표 점을 보관한다.
        self.cached_dynamic_output_points = dynamic_output_points
        self.cached_provisional_output_points = provisional_output_points
        header = Header(stamp=now_msg, frame_id=str(self.base_frame))
        self.publisher.publish(
            point_cloud2.create_cloud_xyz32(header, output_points))
        return stopping_tracks

    def publish_dynamic_heartbeat(self):
        """Collision Monitor가 source timeout으로 정지를 임의 해제하지 않게 한다."""
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=str(self.base_frame))
        output_points = (
            list(self.cached_dynamic_output_points)
            + list(self.cached_provisional_output_points))
        self.publisher.publish(
            point_cloud2.create_cloud_xyz32(
                header, output_points))

    def process_cloud(self, msg, base_to_odom):
        points_base = self.read_xyz(msg)
        stamp = Time.from_msg(msg.header.stamp)
        points_odom = apply_transform(points_base, *base_to_odom)
        clusters = self.cluster(points_base, points_odom)
        now = self.get_clock().now()
        self.associate_and_update(
            clusters, stamp.nanoseconds, now.nanoseconds)

        odom_to_base = self.tracking_to_current_base_matrix()
        if odom_to_base is None:
            return
        stopping_tracks = self.publish_dynamic_cloud(now.to_msg(), odom_to_base)

        if now.nanoseconds - self.last_status_ns >= 500_000_000:
            dynamic_ids = [t.track_id for t in self.tracks.values() if t.dynamic]
            status = String()
            status.data = (
                f'tracks={len(self.tracks)} dynamic={dynamic_ids} '
                f'stopping={stopping_tracks}')
            self.status_publisher.publish(status)
            self.last_status_ns = now.nanoseconds

    def process_pending_clouds(self):
        """스캔 시각의 ego odometry가 도착하면 cloud를 순서대로 처리한다."""
        if self.processing_clouds:
            return
        self.processing_clouds = True
        try:
            while self.pending_clouds:
                msg, queued_ns = self.pending_clouds[0]
                stamp = Time.from_msg(msg.header.stamp)
                if msg.header.frame_id != str(self.base_frame):
                    base_to_odom = None
                    now_ns = self.get_clock().now().nanoseconds
                    if now_ns - self.last_tf_warning_ns > 2_000_000_000:
                        self.get_logger().warning(
                            f'Expected cloud frame {self.base_frame}, got '
                            f'{msg.header.frame_id}')
                        self.last_tf_warning_ns = now_ns
                else:
                    base_to_odom = self.base_to_tracking_matrix(stamp.nanoseconds)
                if base_to_odom is None:
                    now_ns = self.get_clock().now().nanoseconds
                    age = (now_ns - queued_ns) / 1e9
                    if age < float(self.cloud_wait_timeout):
                        break
                    self.pending_clouds.popleft()
                    if now_ns - self.last_queue_warning_ns > 2_000_000_000:
                        self.get_logger().warning(
                            f'Dropping cloud after waiting {age:.2f}s for ego '
                            'odometry at the scan timestamp')
                        self.last_queue_warning_ns = now_ns
                    continue

                self.pending_clouds.popleft()
                self.process_cloud(msg, base_to_odom)
        finally:
            self.processing_clouds = False

    def on_cloud(self, msg):
        # 병합 cloud는 늦게 도착하므로 제한된 queue에서 해당 스캔 시각의
        # ego odometry와 짝을 맞춘다.
        if len(self.pending_clouds) >= int(self.cloud_queue_size):
            self.pending_clouds.popleft()
        self.pending_clouds.append((msg, self.get_clock().now().nanoseconds))
        self.process_pending_clouds()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTrackerHY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
