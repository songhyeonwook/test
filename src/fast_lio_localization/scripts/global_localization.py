#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('accept_initialpose_on_failure', True)
        self.declare_parameter('continuous_global_localization', False)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)
        # body(IMU) 에서 본 차량 기준점. tf_2d 와 같은 값 (mid360.yaml).
        self.declare_parameter('ref_from_body_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('ref_from_body_rpy', [0.0, 0.0, 0.0])
        # ICP 결과로 나온 차량 자세의 roll/pitch 가 이보다 크면 뒤집힌/기운
        # 국소해로 보고 거부한다.
        self.declare_parameter('max_tilt_deg', 20.0)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.accept_initialpose_on_failure = self.get_parameter(
            'accept_initialpose_on_failure').value
        self.continuous_global_localization = self.get_parameter(
            'continuous_global_localization').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value
        self.max_tilt_deg      = self.get_parameter('max_tilt_deg').value
        xyz = self.get_parameter('ref_from_body_xyz').value
        rpy = self.get_parameter('ref_from_body_rpy').value
        self.T_body_ref = tf_transformations.euler_matrix(*rpy)
        self.T_body_ref[:3, 3] = xyz
        # 초기화 전 기본값. 항등으로 두면 camera_init(옆으로 누운 IMU) 그대로라
        # /cur_scan_in_map 이 세로로 서 보인다. 시동 시 차량이 수평이라 가정하고
        # 차량 기준점이 map 원점에 수평으로 놓이는 변환을 기본으로 쓴다.
        self.T_map_to_odom = np.linalg.inv(self.T_body_ref)
        self.pending_init_pose = None

        # ─── State Variables ─────────────────────────────────────
        self.global_map    = None
        self.initialized   = False
        self.cur_odom      = None
        self.cur_scan      = None
        self.localization_timer = None

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array"""
        pts = []
        for x, y, z in pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=True):
            pts.append((x, y, z))
        return np.array(pts, dtype=np.float32)

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.global_map = self.voxel_down_sample(pcd, self.map_voxel_size)
        self.get_logger().info('Global map received and downsampled.')
        self.destroy_subscription(self._map_sub)
        self.try_pending_init_pose()

    def ready(self):
        return (self.global_map is not None and self.cur_scan is not None
                and self.cur_odom is not None)

    def try_pending_init_pose(self):
        # 지도/스캔/오도메트리가 준비되기 전에 들어온 /initialpose 는 버리지
        # 않고 보관했다가 여기서 적용한다 (RViz 에서 너무 일찍 찍는 경우).
        if self.pending_init_pose is not None and self.ready():
            msg, self.pending_init_pose = self.pending_init_pose, None
            self.get_logger().info('Applying /initialpose received before startup finished.')
            self.cb_init_pose(msg)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if not self.ready():
            missing = [name for name, ok in (('global map', self.global_map is not None),
                                             ('first scan', self.cur_scan is not None),
                                             ('odometry', self.cur_odom is not None)) if not ok]
            self.get_logger().warn(
                f'Waiting for {", ".join(missing)}; /initialpose will be applied when ready.')
            self.pending_init_pose = msg
            return

        had_timer = self.localization_timer is not None
        if had_timer:
            self.localization_timer.cancel()

        if self.initialized:
            self.get_logger().info('Received new /initialpose, retrying global localization.')
        else:
            self.get_logger().info('Received initial /initialpose, starting global localization.')

        # /initialpose 는 차량(base_link=지면, roll=pitch=0) 자세다. FAST-LIO 의
        # body 는 옆으로 누운 IMU 프레임이므로 ref_from_body 를 끼워서
        # map->camera_init 초기 추정을 만든다. 이걸 빼면(옛 TF 시절 코드) 초기
        # 추정이 roll 90도 틀려서 ICP 가 바닥/천장이 뒤집힌 해로 수렴한다.
        T_map_ref = self.pose_to_mat(msg)
        T_cam_ref = self.pose_to_mat(self.cur_odom) @ self.T_body_ref
        initial_map_to_odom = T_map_ref @ np.linalg.inv(T_cam_ref)

        success = self.global_localization(initial_map_to_odom)
        if success:
            self.initialized = True
            self.ensure_localization_timer()
            self.get_logger().info('Global localization initialized / updated successfully.')
        else:
            if self.accept_initialpose_on_failure:
                self.T_map_to_odom = initial_map_to_odom
                self.publish_map_to_odom(self.T_map_to_odom)
                self.initialized = True
                self.ensure_localization_timer()
                self.get_logger().warn(
                    'ICP failed, but accepted manual /initialpose as provisional map->odom estimate.')
            else:
                if had_timer and self.localization_timer is not None:
                    self.localization_timer.reset()
                self.get_logger().warn(
                    'Re-localization from /initialpose failed, keeping previous state.')

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg
        self.try_pending_init_pose()

    def cb_save_cur_scan(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        if len(pts) == 0:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd
        self.try_pending_init_pose()

        # /cloud_registered 는 camera_init 프레임인데 그 프레임은 TF 에 없다
        # (디버그 전용). 현재 map->camera_init 추정을 직접 곱해서 이름대로
        # map 프레임 클라우드로 발행한다. RViz 정합 확인용.
        hom = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=np.float32)])
        pts_map = (self.T_map_to_odom @ hom.T).T[:, :3]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        self.pub_pc_in_map.publish(pc2.create_cloud_xyz32(header, pts_map))

    def timer_callback(self):
        self.global_localization(self.T_map_to_odom)

    def ensure_localization_timer(self):
        if not self.continuous_global_localization:
            if self.localization_timer is not None:
                self.localization_timer.cancel()
            self.get_logger().info(
                'Continuous global localization is disabled; keeping the current manual/ICP map->odom estimate.')
            return

        if self.localization_timer is None:
            period = 1.0 / self.freq_localization
            self.localization_timer = self.create_timer(period, self.timer_callback)
        else:
            self.localization_timer.reset()

    # (voxel 배율, 대응거리 m, 최대 반복). 거칠게 잡고 점점 조인다.
    # 예전 2단계 x 20회는 초기 추정이 2m/15도 틀리면 fitness 0.8 짜리 얕은
    # 국소해에서 멈췄다 (bag 으로 확인). 마지막 0.4m 단계가 포즈를 조인다.
    ICP_STAGES = [(5, 5.0, 50), (2, 2.0, 50), (1, 1.0, 100), (1, 0.4, 100)]

    def global_localization(self, pose_est):
        self.get_logger().info('Performing global localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)
        odom = self.cur_odom

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, odom)

        T = pose_est
        for voxel_scale, corr_dist, max_iter in self.ICP_STAGES:
            T, tight_fitness = self.registration(
                scan_copy, submap, T, voxel_scale, corr_dist, max_iter)
        # 수락 판정은 예전과 같은 1m 대응거리 fitness (localization_th 의미 유지)
        fitness = self.evaluate(scan_copy, submap, T, 1.0)
        self.get_logger().info(
            f'ICP fitness: {fitness:.3f} (1.0m), {tight_fitness:.3f} (0.4m)')

        # 뒤집힌/기운 해 거부. 실내 공간은 바닥/천장, 좌/우가 바뀐 해도
        # fitness 가 높게 나올 수 있어서 차량 자세의 roll/pitch 로 걸러낸다.
        T_map_ref = T @ self.pose_to_mat(odom) @ self.T_body_ref
        roll, pitch, _ = tf_transformations.euler_from_matrix(T_map_ref)
        tilt = np.degrees(max(abs(roll), abs(pitch)))
        if tilt > self.max_tilt_deg:
            self.get_logger().warn(
                f'Global localization rejected: vehicle roll/pitch {tilt:.1f} deg '
                f'(flipped or tilted solution).')
            return False

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            self.publish_map_to_odom(T)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    def publish_map_to_odom(self, T):
        odom = Odometry()
        xyz  = tf_transformations.translation_from_matrix(T)
        quat = tf_transformations.quaternion_from_matrix(T)
        odom.pose.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom.header.stamp = self.cur_odom.header.stamp if self.cur_odom is not None else \
            self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        self.pub_map_to_odom.publish(odom)

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = np.asarray(self.global_map.points)
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        if self.FOV >= 2*np.pi:
            mask = (pts_scan[:,0] < self.FOV_FAR)
        else:
            ang  = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            mask = (pts_scan[:,0]>0)&(pts_scan[:,0]<self.FOV_FAR)&(np.abs(ang)<self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(subpts)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration(self, scan, submap, initial, voxel_scale, corr_dist, max_iter):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size * voxel_scale)
        reg = o3d.pipelines.registration.registration_icp(
            down(scan), down(submap),
            max_correspondence_distance=corr_dist,
            init=initial,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        )
        return reg.transformation, reg.fitness

    def evaluate(self, scan, submap, T, corr_dist):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size)
        return o3d.pipelines.registration.evaluate_registration(
            down(scan), down(submap), corr_dist, T).fitness

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
