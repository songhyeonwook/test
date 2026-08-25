#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import TransformStamped


class Tf2DBridge(Node):
    def __init__(self):
        super().__init__('tf_2d_bridge')

        # ===== Parameters =====
        self.declare_parameter('parent_3d', 'map')
        self.declare_parameter('odom_2d', 'odom')
        self.declare_parameter('base_2d', 'base_link')
        self.declare_parameter('caminit', 'camera_init')
        self.declare_parameter('body', 'body')
        self.declare_parameter('rate_hz', 50.0)

        self.parent_3d = self.get_parameter('parent_3d').value
        self.odom_2d = self.get_parameter('odom_2d').value
        self.base_2d = self.get_parameter('base_2d').value
        self.caminit = self.get_parameter('caminit').value
        self.body = self.get_parameter('body').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # ===== TF =====
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.02
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'tf_2d_bridge started: {self.parent_3d}->{self.caminit}->{self.body} '
            f' ==> {self.parent_3d}->{self.odom_2d}->{self.base_2d}'
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
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
        ], dtype=float)
        return R

    @staticmethod
    def transform_to_matrix(t: TransformStamped):
        """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix"""
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        M = np.eye(4, dtype=float)
        M[:3, :3] = Tf2DBridge.quat_to_rot(qx, qy, qz, qw)
        M[0, 3] = tx
        M[1, 3] = ty
        M[2, 3] = tz
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

    # ---------- main loop ----------
    def on_timer(self):
        try:
            # Read 3D TF chain
            t_parent_cam = self.tf_buffer.lookup_transform(
                self.parent_3d,
                self.caminit,
                Time(),
                timeout=Duration(seconds=0.1)
            )
            t_cam_body = self.tf_buffer.lookup_transform(
                self.caminit,
                self.body,
                Time(),
                timeout=Duration(seconds=0.1)
            )

            M_parent_cam = self.transform_to_matrix(t_parent_cam)
            M_cam_body = self.transform_to_matrix(t_cam_body)
            M_parent_body = M_parent_cam @ M_cam_body

            # Planarize to 2D
            M_parent_odom2d, q_parent_odom2d = self.planarize(M_parent_cam)
            M_parent_base2d, _ = self.planarize(M_parent_body)

            # Compute odom -> base_link
            M_odom_base2d = np.linalg.inv(M_parent_odom2d) @ M_parent_base2d
            yaw_odom_base = self.yaw_from_matrix(M_odom_base2d)
            q_odom_base2d = self.quat_from_yaw(yaw_odom_base)

            now = self.get_clock().now().to_msg()

            # Publish 2D TFs for Nav2
            self.publish_tf(
                M_parent_odom2d,
                q_parent_odom2d,
                self.parent_3d,
                self.odom_2d,
                now
            )
            self.publish_tf(
                M_odom_base2d,
                q_odom_base2d,
                self.odom_2d,
                self.base_2d,
                now
            )

        except Exception:
            # TF가 아직 안 올라왔거나 잠깐 lookup 실패하면 다음 주기에 재시도
            pass


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