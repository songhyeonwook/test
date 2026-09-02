#!/usr/bin/env python3
"""/docking 토픽으로 top 라이다 TF 를 주행/도킹모드 사이에서 전환한다.

top 라이다는 리프트에 달려 있어 z 로 움직인다. 리프트 보드는 위치 피드백이
없으므로(개방루프, lift_node 참조) 연속 TF 는 낼 수 없고, 양 끝 자세를 바닥
평면 실측으로 잡아 둔 뒤 모드 신호로 전환한다.

  /docking  std_msgs/Bool  (구독)  true=도킹모드, false=주행모드(기본)

값은 mount.xacro 한 곳에서 읽는다 (top_xyz/top_rpy = 주행, top_dock_* = 도킹).
robot_state_publisher 도 URDF 조인트로 주행모드 livox_frame->livox_top 을
latch 하므로, 이 노드는 1초마다 재발행해서 늦게 붙는 리스너도 항상 이 노드의
값으로 수렴하게 한다 (tf_static 은 마지막 수신이 이긴다).

주의: 도킹모드에서는 FAST-LIO 의 extrinsic(mid360.yaml, 주행모드 유도값)이
실물과 0.6 m 어긋난다. 리프트가 올라간 동안의 FAST-LIO 측위는 믿지 말 것.
"""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

XACRO_NS = '{http://www.ros.org/wiki/xacro}'


def mount_vec(props, name):
    """'a b ${pi/2}' 꼴 property 를 [float]*3 으로. ${} 안은 pi 만 허용."""
    vals = []
    for tok in props[name].split():
        if tok.startswith('${') and tok.endswith('}'):
            vals.append(float(eval(tok[2:-1], {'__builtins__': {}}, {'pi': math.pi})))
        else:
            vals.append(float(tok))
    if len(vals) != 3:
        raise ValueError(f'{name} 는 3개 값이어야 한다: {props[name]!r}')
    return vals


def load_poses():
    path = os.path.join(get_package_share_directory('navigation'),
                        'urdf', 'mount.xacro')
    props = {p.get('name'): p.get('value')
             for p in ET.parse(path).getroot().iter(f'{XACRO_NS}property')}
    missing = [k for k in ('top_xyz', 'top_rpy', 'top_dock_xyz', 'top_dock_rpy')
               if k not in props]
    if missing:
        raise SystemExit(
            f'mount.xacro 에 {missing} property 가 없다. 주석 처리돼 있으면 '
            f'해제할 것 - 모드 전환은 이 노드가 하므로 두 벌 다 살아 있어야 한다.')
    return {
        False: (mount_vec(props, 'top_xyz'), mount_vec(props, 'top_rpy')),
        True: (mount_vec(props, 'top_dock_xyz'), mount_vec(props, 'top_dock_rpy')),
    }


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class TopTfSwitcher(Node):
    def __init__(self):
        super().__init__('top_tf_switcher')
        self.poses = load_poses()
        self.docking = False
        self.broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(Bool, '/docking', self.on_docking, 1)
        self.create_timer(1.0, self.publish)
        self.publish()
        self.get_logger().info('시작: 주행모드 (/docking 토픽 대기)')

    def on_docking(self, msg):
        if bool(msg.data) != self.docking:
            self.docking = bool(msg.data)
            self.get_logger().info(
                f"top TF 전환 -> {'도킹' if self.docking else '주행'}모드")
        self.publish()

    def publish(self):
        xyz, rpy = self.poses[self.docking]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'livox_frame'
        t.child_frame_id = 'livox_top'
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        qx, qy, qz, qw = quat_from_rpy(*rpy)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TopTfSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
