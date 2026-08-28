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
        # 주기 보정 1회에 허용할 map->odom 변화량. map->odom 은 FAST-LIO 의
        # 드리프트를 깎는 보정값이라 한 주기에 조금씩만 움직이는 게 정상이다.
        # 이보다 크게 뛰면 반복 구조에 한 칸 밀려 정합된 해로 본다.
        self.declare_parameter('max_correction_xyz', 0.5)   # m
        self.declare_parameter('max_correction_deg', 5.0)   # deg

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
        self.max_correction_xyz = self.get_parameter('max_correction_xyz').value
        self.max_correction_deg = self.get_parameter('max_correction_deg').value
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
        # /initialpose 가 ICP 실패로 임시 채택된 상태. 아직 수 m 틀려 있을 수
        # 있으니 다음 주기는 추적용이 아니라 초기화 사다리로 한 번 더 돌린다.
        self.needs_reinit  = False

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
        """PointCloud2 → (N×3) NumPy array

        점마다 파이썬 루프를 돌면 30k 점 스캔 하나에 0.15초가 걸린다(보드 실측).
        /cloud_registered 가 10Hz 로 들어오므로 그것만으로 단일 스레드
        실행기가 포화되고, 같은 스레드에 있는 ICP 타이머가 굶어서
        freq_localization 을 올려도 그만큼 돌지 않는다. 같은 일을 100배 빠르게 한다.
        """
        pts = pc2.read_points_numpy(
            pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.asarray(pts, dtype=np.float32).reshape(-1, 3)

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

        success = self.global_localization(initial_map_to_odom, self.ICP_STAGES_INIT)
        if success:
            self.initialized = True
            self.ensure_localization_timer()
            self.get_logger().info('Global localization initialized / updated successfully.')
        else:
            if self.accept_initialpose_on_failure:
                self.T_map_to_odom = initial_map_to_odom
                self.publish_map_to_odom(self.T_map_to_odom)
                self.initialized = True
                self.needs_reinit = True
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
        if self.needs_reinit:
            # 임시 채택된 수동 포즈에서 출발. 아직 크게 틀렸을 수 있으므로
            # 초기화 사다리를 쓰고 보정량 제한도 걸지 않는다.
            if self.global_localization(self.T_map_to_odom, self.ICP_STAGES_INIT):
                self.needs_reinit = False
            return
        self.global_localization(
            self.T_map_to_odom, self.ICP_STAGES_TRACK, tracking=True)

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
    # 초기화용. 수동 /initialpose 는 2m/15도씩 틀릴 수 있어서 5m 부터 훑는다.
    # 예전 2단계 x 20회는 초기 추정이 2m/15도 틀리면 fitness 0.8 짜리 얕은
    # 국소해에서 멈췄다 (bag 으로 확인). 마지막 0.4m 단계가 포즈를 조인다.
    ICP_STAGES_INIT = [(5, 5.0, 50), (2, 2.0, 50), (1, 1.0, 100), (1, 0.4, 100)]
    # 추적용. 직전 추정이 이미 수 cm 안이다. 여기에 초기화 사다리를 그대로
    # 쓰면(예전 코드) 매 주기 5m 대응거리부터 다시 훑게 되고, 복도나 같은
    # 간격 기둥 같은 반복 구조에서 한 칸 밀린 정합으로 끌려간다. 그 해도
    # 1m fitness 는 여유롭게 통과하므로 그대로 채택되어 포즈가 튄다.
    ICP_STAGES_TRACK = [(1, 1.0, 50), (1, 0.4, 100)]

    def global_localization(self, pose_est, stages, tracking=False):
        self.get_logger().info('Performing global localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)
        odom = self.cur_odom

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, odom)

        T = pose_est
        for voxel_scale, corr_dist, max_iter in stages:
            T, _ = self.registration(
                scan_copy, submap, T, voxel_scale, corr_dist, max_iter)
        # 수락 판정은 예전과 같은 1m 대응거리 fitness (localization_th 의미 유지).
        # 0.4m 쪽은 같은 클라우드로 재평가해서 1m 값과 직접 비교할 수 있게 한다
        # (예전엔 마지막 ICP 단계의 내부 fitness 라 다운샘플 배율이 달랐다).
        fitness = self.evaluate(scan_copy, submap, T, 1.0)
        tight_fitness = self.evaluate(scan_copy, submap, T, 0.4)
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

        if tracking:
            # 1m fitness 는 실내에서 포즈가 0.9m 틀려도 거의 1.0 이 나와서
            # 튄 해를 못 거른다. 추적 중에는 아래 두 가지로 판정한다.

            # (a) 보정량 제한. map->odom 은 드리프트 보정이라 한 주기에
            #     조금씩만 변해야 한다. 크게 뛰면 잘못 정합된 해다.
            d_xyz, d_deg = self.pose_delta(pose_est, T)
            if d_xyz > self.max_correction_xyz or d_deg > self.max_correction_deg:
                self.get_logger().warn(
                    f'Global localization rejected: correction too large '
                    f'({d_xyz:.2f} m, {d_deg:.1f} deg); keeping previous estimate.')
                return False

            # (b) 지금 쓰고 있는 추정보다 나빠지면 버린다. 절대 문턱과 달리
            #     맵 커버리지(사람, 치워진 가구, 미측량 구역)에 영향받지 않는
            #     같은 스캔/서브맵 위의 비교라 따로 튜닝할 값이 없다.
            base_tight = self.evaluate(scan_copy, submap, pose_est, 0.4)
            if tight_fitness < base_tight:
                self.get_logger().warn(
                    f'Global localization rejected: 0.4m fitness got worse '
                    f'({base_tight:.3f} -> {tight_fitness:.3f}); keeping previous estimate.')
                return False

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            self.publish_map_to_odom(T)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    @staticmethod
    def pose_delta(T_a, T_b):
        """두 변환의 차이를 (이동 m, 회전 deg) 로 돌려준다."""
        d = np.linalg.inv(T_a) @ T_b
        dist = float(np.linalg.norm(d[:3, 3]))
        cos = (np.trace(d[:3, :3]) - 1.0) / 2.0
        ang = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
        return dist, ang

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
            # 360도 라이다는 앞뒤 구분이 없으니 반경으로 자른다. 예전 x < FOV_FAR 는
            # -x 와 y/z 가 무제한이라 사실상 맵 전체를 서브맵으로 넘겼고,
            # 그래서 ICP 가 느리고 1단계(5m 대응거리)에서 먼 구조물에 끌렸다.
            mask = (np.linalg.norm(pts_scan[:, :3], axis=1) < self.FOV_FAR)
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
