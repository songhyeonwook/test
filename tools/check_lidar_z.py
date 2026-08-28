#!/usr/bin/env python3
"""라이다 장착 z 를 수평면(천장/바닥)으로 검증한다.

z 는 벽으로는 잡히지 않는다. 벽은 수직면이라 위아래로 미끄러져도 잔차가 안
늘어난다. 그래서 check_lidar_overlap.py 의 ICP 는 z 에 대해 사실상 눈이 멀었고,
이 환경에서는 서로 다른 면을 보는 두 클라우드를 억지로 끌어당겨 엉뚱한 최소점에
빠진다(front/rear 처럼 실제로는 맞는 쌍도 1.3 m 어긋난다고 보고했다).

이 도구는 대신 '수평면 하나를 두 라이다가 같은 높이로 보는가' 만 본다.
front/rear (같은 높이, 원점대칭)를 기준으로 삼아 천장 평면을 맞추고, 검사 대상
라이다의 천장 점이 그 평면 위/아래로 얼마나 떨어져 있는지를 잰다. 그 값이 곧
장착 z 오차다.

  (센서 + description 이 떠 있는 상태에서)
  source install/setup.bash
  python3 tools/check_lidar_z.py            # top 검사 (기본)
  python3 tools/check_lidar_z.py rear       # rear 를 front 기준으로 검사

천장이 없는 곳(야외)에서는 못 쓴다. 바닥을 쓰려면 --plane floor.
주의: 잔차의 사분위 산포(p25~p75)가 넓으면 z 가 아니라 rpy 가 틀린 것이다.
"""
import argparse
import sys
import time

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

LIDARS = {
    'front': '/livox/lidar_192_168_2_102',
    'rear': '/livox/lidar_192_168_3_144',
    'top': '/livox/lidar_192_168_1_135',
}
TARGET = 'base_link'
FRAMES = 30
BANDS = {'ceiling': (2.0, 3.6), 'floor': (-0.6, 0.4)}


class Collector(Node):
    def __init__(self, names):
        super().__init__('lidar_z_check')
        self.buf = {k: [] for k in names}
        self.frame_id = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        for k in names:
            self.create_subscription(
                PointCloud2, LIDARS[k],
                lambda m, k=k: self.cb(m, k), qos_profile_sensor_data)

    def cb(self, msg, k):
        if len(self.buf[k]) >= FRAMES:
            return
        self.frame_id[k] = msg.header.frame_id
        a = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                     skip_nans=True)
        self.buf[k].append(
            np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64))

    def done(self):
        return all(len(v) >= FRAMES for v in self.buf.values())

    def lookup(self, frame):
        tf = self.tf_buffer.lookup_transform(
            TARGET, frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=5.0))
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        M = np.eye(4)
        M[:3, :3] = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        M[:3, 3] = [t.x, t.y, t.z]
        return M


def to_base_link(frames, M):
    p = np.vstack(frames)
    p = p[np.linalg.norm(p, axis=1) > 0.5]
    hom = np.hstack([p, np.ones((len(p), 1))])
    return (M @ hom.T).T[:, :3]


def plane_points(pts, lo, hi):
    """z 대역 안에서 가장 큰 평면의 점들과 평면계수를 돌려준다."""
    c = pts[(pts[:, 2] > lo) & (pts[:, 2] < hi)]
    if len(c) < 500:
        return None, None
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(c)
    pc = pc.voxel_down_sample(0.04)
    model, inl = pc.segment_plane(0.04, 3, 5000)
    return np.asarray(pc.points)[inl], model


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', nargs='?', default='top',
                    choices=[k for k in LIDARS],
                    help='검사할 라이다 (기본 top)')
    ap.add_argument('--plane', default='ceiling', choices=list(BANDS),
                    help='기준으로 쓸 수평면 (기본 ceiling)')
    args = ap.parse_args()

    # front/rear 는 같은 높이라 둘을 합쳐 기준으로 쓴다. 검사 대상이 그중
    # 하나면 나머지 하나만 기준이 된다.
    ref_names = [k for k in ('front', 'rear') if k != args.target]
    if not ref_names:
        print('front 와 rear 를 동시에 검사할 수는 없다.')
        return 1
    names = sorted(set(ref_names + [args.target]))

    rclpy.init()
    n = Collector(names)
    print(f'{FRAMES} 프레임씩 수집 중... (차량 정지)')
    t0 = time.time()
    while not n.done() and time.time() - t0 < 40.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    counts = {k: len(v) for k, v in n.buf.items()}
    if not all(counts.values()):
        print('데이터가 안 온다:', counts)
        rclpy.shutdown()
        return 1
    try:
        mats = {k: n.lookup(n.frame_id[k]) for k in names}
    except tf2_ros.TransformException as e:
        print(f'TF 조회 실패: {e}')
        print('description.launch.py 가 떠 있는지 확인할 것.')
        return 1
    finally:
        rclpy.shutdown()

    lo, hi = BANDS[args.plane]
    ref = np.vstack([to_base_link(n.buf[k], mats[k]) for k in ref_names])
    ref_p, model = plane_points(ref, lo, hi)
    if ref_p is None:
        print(f'기준 라이다에서 {args.plane} 평면을 못 찾았다.')
        return 1
    a, b, c, d = model
    if c < 0:
        a, b, c, d = -a, -b, -c, -d
    nrm = np.linalg.norm([a, b, c])
    a, b, c, d = a / nrm, b / nrm, c / nrm, d / nrm
    tilt = np.degrees(np.arccos(np.clip(c, -1, 1)))
    print(f'기준({"+".join(ref_names)}) {args.plane}: {len(ref_p)} 점, '
          f'기울기 {tilt:.2f} deg, 높이 {-d / c:.4f} m')

    tgt = to_base_link(n.buf[args.target], mats[args.target])
    tgt_p, _ = plane_points(tgt, lo, hi)
    if tgt_p is None:
        print(f'{args.target} 에서 {args.plane} 평면을 못 찾았다.')
        return 1

    # 천장이 조금 기울어 있어도 되도록 겹치는 xy 영역에서만 비교한다
    p_lo = np.maximum(ref_p[:, :2].min(0), tgt_p[:, :2].min(0))
    p_hi = np.minimum(ref_p[:, :2].max(0), tgt_p[:, :2].max(0))

    def clip(p):
        return p[(p[:, 0] > p_lo[0]) & (p[:, 0] < p_hi[0]) &
                 (p[:, 1] > p_lo[1]) & (p[:, 1] < p_hi[1])]

    r2, t2 = clip(ref_p), clip(tgt_p)
    if len(t2) < 200 or len(r2) < 200:
        print(f'겹치는 영역의 점이 너무 적다 (기준 {len(r2)}, {args.target} {len(t2)}).')
        return 1
    print(f'겹치는 xy 영역 x[{p_lo[0]:+.2f},{p_hi[0]:+.2f}] '
          f'y[{p_lo[1]:+.2f},{p_hi[1]:+.2f}]  '
          f'기준 {len(r2)} 점 / {args.target} {len(t2)} 점')

    def signed(p):
        return p @ np.array([a, b, c]) + d

    print(f'\n기준 점의 평면 잔차       : 중앙 {np.median(signed(r2)):+.4f} m'
          f'  (0 이어야 정상)')
    sd = signed(t2)
    off = float(np.median(sd))
    spread = float(np.percentile(sd, 75) - np.percentile(sd, 25))
    print(f'{args.target} 점의 평면 잔차 : 중앙 {off:+.4f} m'
          f'  (p25 {np.percentile(sd, 25):+.4f}, p75 {np.percentile(sd, 75):+.4f})')

    cur = mats[args.target][2, 3]
    ok = abs(off) < 0.02
    print(f'\n현재 {args.target} 높이(base_link) = {cur:.4f} m')
    if ok:
        print(f'=> z 는 맞는다 (잔차 {off:+.4f} m)')
    else:
        print(f'=> z 가 {off:+.4f} m 어긋났다. 올바른 높이 = {cur - off:.4f} m')
        print(f'   mount.xacro 의 {args.target}_xyz z (livox_frame 0.32 기준) = '
              f'{cur - off - 0.32:.4f}')
    if spread > 0.06:
        print(f'\n주의: 잔차 산포가 {spread:.3f} m 로 넓다. z 가 아니라 '
              f'{args.target}_rpy 가 틀렸다는 뜻이다')
        print(f'      (겹치는 영역 폭에 대해 약 '
              f'{np.degrees(np.arctan(spread / max(p_hi[0] - p_lo[0], 1e-6))):.1f} deg).')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
