#!/usr/bin/env python3
# coding=utf-8

import copy
import threading

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        # FAST-LIO Odometry 자체가 약 10 Hz다. 같은 행렬 분해 결과를 50 Hz로
        # 반복 계산하면 Jetson에서 CPU만 크게 사용하고 LiDAR merge 콜백을
        # 밀어낸다. 3D localization은 새 Odometry 주기와 맞춰 내보낸다.
        self.declare_parameter('publish_rate_hz', 10.0)
        self.FREQ_PUB_LOCALIZATION = float(
            self.get_parameter('publish_rate_hz').value)
        if self.FREQ_PUB_LOCALIZATION <= 0.0:
            self.get_logger().warning(
                'publish_rate_hz must be positive; using 10.0 Hz')
            self.FREQ_PUB_LOCALIZATION = 10.0

        # map->camera_init TF. 정식 트리(map->odom->base_link)는 tf_2d.py 가
        # 내므로 평소에는 끈다. RViz 에서 camera_init 프레임 클라우드
        # (/cloud_registered 등)를 map 위에 겹쳐 볼 때만 켠다.
        self.declare_parameter('publish_debug_tf', False)
        self.publish_debug_tf = bool(
            self.get_parameter('publish_debug_tf').value)

        self.lock = threading.Lock()
        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        # map -> body (3D, 라이다 IMU 프레임). 2D 평면 자세 /localization (map -> 지면)
        # 은 tf_2d.py 가 낸다. 이름이 겹치지 않도록 여기서는 /localization_3d 로 낸다.
        self.declare_parameter('localization_topic', '/localization_3d')
        self.pub_localization = self.create_publisher(
            Odometry, self.get_parameter('localization_topic').value, 1)
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info('Transform Fusion Node Initialized')

    def cb_save_cur_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom = msg

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot

    def timer_callback(self):
        # 평소 Nav2는 tf_2d_bridge의 map->odom->base_link와 /localization을
        # 사용한다. /localization_3d 구독자도 debug TF도 없으면 아래의 NumPy/
        # tf_transformations 행렬 분해 결과는 아무도 소비하지 않는다. Jetson의
        # LiDAR 병합 콜백을 밀지 않도록 실제 소비자가 있을 때만 계산한다.
        if (not self.publish_debug_tf and
                self.pub_localization.get_subscription_count() == 0):
            return

        with self.lock:
            odom    = copy.deepcopy(self.cur_odom_to_baselink)
            map2odom = copy.deepcopy(self.cur_map_to_odom)
        if odom is None:
            return

        # map -> odom
        T_map_to_odom = self.pose_to_mat(map2odom) if map2odom is not None else np.eye(4)
        trans = tf_transformations.translation_from_matrix(T_map_to_odom)
        quat  = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        # 디버그 tf 브로드캐스트
        if self.publish_debug_tf:
            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id    = 'map'
            t_msg.child_frame_id     = 'camera_init'
            t_msg.transform.translation.x = trans[0]
            t_msg.transform.translation.y = trans[1]
            t_msg.transform.translation.z = trans[2]
            t_msg.transform.rotation.x    = quat[0]
            t_msg.transform.rotation.y    = quat[1]
            t_msg.transform.rotation.z    = quat[2]
            t_msg.transform.rotation.w    = quat[3]
            self.tf_broadcaster.sendTransform(t_msg)

        # fused localization
        T_odom_to_base = self.pose_to_mat(odom)
        T_map_to_base  = T_map_to_odom @ T_odom_to_base
        xyz   = tf_transformations.translation_from_matrix(T_map_to_base)
        quat2 = tf_transformations.quaternion_from_matrix(T_map_to_base)

        loc_msg = Odometry()
        loc_msg.header.stamp          = odom.header.stamp
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'body'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = odom.twist
        self.pub_localization.publish(loc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
