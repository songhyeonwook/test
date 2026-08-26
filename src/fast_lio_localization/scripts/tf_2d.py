#!/usr/bin/env python3
"""FAST-LIO 3D 오도메트리를 Nav2 용 2D TF 로 변환한다.

TF lookup 없이 토픽만 구독한다. camera_init / body 는 TF 프레임으로 쓰지 않는다.

  구독  /Odometry     camera_init -> body   (FAST-LIO, 라이다 주기 ~10Hz)
        /map_to_odom  map -> camera_init 보정 (global_localization, 저주기)

  발행  map -> odom        planarize(map->camera_init)  보정(점프)을 흡수
        odom -> base_link  나머지 연속 오도메트리

body 는 차량 기준점이 아니라 옆으로 누운 상단 라이다의 IMU 프레임이라 그대로
평면화하면 yaw 가 엉킨다. ref_from_body_* (body -> 차량 정렬 기준점, mid360.yaml)
를 곱한 뒤 평면화한다. 값 정합은 tools/check_frames.py 가 검사한다.

TF 는 오도메트리 stamp 로 내고, 라이다 주기(~10Hz)가 Nav2 transform_tolerance
(0.1s)에 빠듯하므로 keepalive 타이머가 최신값을 현재 시각으로 재발행해 준다.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class Tf2DBridge(Node):
    def __init__(self):
        super().__init__('tf_2d_bridge')

        # ===== Parameters =====
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('map_to_odom_topic', '/map_to_odom')
        # body(IMU) 에서 본 차량 정렬 기준점.
        # 기본값 0 은 자리표시자다. 반드시 mid360.yaml 에서 받아야 한다.
        self.declare_parameter('ref_from_body_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('ref_from_body_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('keepalive_rate_hz', 50.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        map_to_odom_topic = self.get_parameter('map_to_odom_topic').value
        xyz = self.get_parameter('ref_from_body_xyz').value
        rpy = self.get_parameter('ref_from_body_rpy').value
        keepalive_hz = float(self.get_parameter('keepalive_rate_hz').value)

        self.T_body_ref = np.eye(4)
        self.T_body_ref[:3, :3] = self.rpy_to_rot(*rpy)
        self.T_body_ref[:3, 3] = xyz

        self.T_map_cam = np.eye(4)   # /map_to_odom 수신 전에는 항등
        self.last_map_odom = None    # (M, q) 마지막으로 계산한 map->odom
        self.last_odom_base = None   # (M, q) 마지막으로 계산한 odom->base_link

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Odometry, odom_topic, self.cb_odometry, 10)
        self.create_subscription(
            Odometry, map_to_odom_topic, self.cb_map_to_odom, 1)

        if keepalive_hz > 0.0:
            self.create_timer(1.0 / keepalive_hz, self.on_keepalive)

        self.get_logger().info(
            f'tf_2d_bridge started: {odom_topic} + {map_to_odom_topic} '
            f'==> {self.map_frame}->{self.odom_frame}->{self.base_frame}'
        )

    # ---------- math helpers ----------
    @staticmethod
    def quat_to_rot(qx, qy, qz, qw):
        """Quaternion -> 3x3 rotation matrix"""
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        R = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),   1.0 - 2.0 * (xx + yy)],
        ], dtype=float)
        return R

    @staticmethod
    def rpy_to_rot(r, p, y):
        """URDF 관례(Rz*Ry*Rx)의 rpy -> 3x3 rotation matrix"""
        def Rx(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

        def Ry(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

        def Rz(a):
            c, s = math.cos(a), math.sin(a)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)

        return Rz(y) @ Ry(p) @ Rx(r)

    @staticmethod
    def odom_pose_to_matrix(msg: Odometry):
        """nav_msgs/Odometry pose -> 4x4 homogeneous matrix"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        M = np.eye(4, dtype=float)
        M[:3, :3] = Tf2DBridge.quat_to_rot(q.x, q.y, q.z, q.w)
        M[:3, 3] = [p.x, p.y, p.z]
        return M

    @staticmethod
    def yaw_from_matrix(M):
        """4x4 or 3x3 rotation matrix -> yaw"""
        return math.atan2(M[1, 0], M[0, 0])

    @staticmethod
    def quat_from_yaw(yaw):
        """yaw -> quaternion (x,y,z,w)"""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def planarize(M):
        """
        3D homogeneous matrix -> 2D planar homogeneous matrix
        Keep x, y, yaw only. Force z=0, roll=0, pitch=0.
        """
        tx = float(M[0, 3])
        ty = float(M[1, 3])
        yaw = Tf2DBridge.yaw_from_matrix(M)

        c = math.cos(yaw)
        s = math.sin(yaw)

        M2 = np.array([
            [c, -s, 0.0, tx],
            [s,  c, 0.0, ty],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        q = Tf2DBridge.quat_from_yaw(yaw)
        return M2, q

    def publish_tf(self, M, q, parent, child, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(M[0, 3])
        t.transform.translation.y = float(M[1, 3])
        t.transform.translation.z = float(M[2, 3])

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(t)

    def publish_chain(self, stamp):
        M_mo, q_mo = self.last_map_odom
        M_ob, q_ob = self.last_odom_base
        self.publish_tf(M_mo, q_mo, self.map_frame, self.odom_frame, stamp)
        self.publish_tf(M_ob, q_ob, self.odom_frame, self.base_frame, stamp)

    # ---------- callbacks ----------
    def cb_map_to_odom(self, msg: Odometry):
        self.T_map_cam = self.odom_pose_to_matrix(msg)

    def cb_odometry(self, msg: Odometry):
        T_map_cam = self.T_map_cam
        T_cam_body = self.odom_pose_to_matrix(msg)
        T_map_ref = T_map_cam @ T_cam_body @ self.T_body_ref

        # odom = planarize(map->camera_init). 보정이 없으면 map 과 일치한다.
        M_map_odom, q_map_odom = self.planarize(T_map_cam)
        M_map_base, _ = self.planarize(T_map_ref)

        M_odom_base = np.linalg.inv(M_map_odom) @ M_map_base
        q_odom_base = self.quat_from_yaw(self.yaw_from_matrix(M_odom_base))

        self.last_map_odom = (M_map_odom, q_map_odom)
        self.last_odom_base = (M_odom_base, q_odom_base)

        # 센서 데이터와 시간축이 맞도록 오도메트리 stamp 그대로 낸다.
        self.publish_chain(msg.header.stamp)

    def on_keepalive(self):
        # 라이다 주기 사이를 메꾼다. odom 정지 가정으로 최신값을 재발행.
        if self.last_map_odom is None:
            return
        self.publish_chain(self.get_clock().now().to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = Tf2DBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
