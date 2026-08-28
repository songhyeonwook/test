#!/usr/bin/env python3
"""라이다 장착값이 '실물'과 맞는지 검사한다.

check_frames.py 는 설정 파일들끼리 서로 맞는지만 본다. URDF 값 자체가 실제
장착과 다르면 전부 [OK] 로 나오면서 조용히 틀린다. 이 도구는 그 공백을 메운다.
병합을 TF 로 하게 된 뒤로는 URDF 가 유일한 장착값 원본이라 이 검사가 더 중요하다.

원리: MID-360 은 360도라 두 대가 같은 방을 본다. TF 로 base_link 에 옮기면 두
클라우드는 같은 벽 위에 겹쳐야 한다. 겹치지 않으면 URDF 장착값이 실물과 다른
것이고, 그 상태로 병합한 클라우드는 벽이 두 겹으로 찍혀서 FAST-LIO 가 제약이
약한 방향으로 서서히 미끄러진다.

  (센서를 띄운 상태에서 - TF 가 있어야 하므로 description 도 떠 있어야 한다)
  source install/setup.bash
  python3 tools/check_lidar_overlap.py             # front vs rear
  python3 tools/check_lidar_overlap.py front top   # front vs top

주의: 실내 잡동사니가 많고 두 라이다가 서로 다른 면을 보는 곳에서는 이 ICP 가
믿을 게 못 된다. 2026-08-28 실측에서 front/rear (천장 평면으로는 1.3 mm 로
일치하는 쌍) 에 대해서도 "1.3 m / 8.3 deg 어긋남" 을 보고했다. 초기 최근접거리
중앙값이 0.3 m 를 넘거나 fitness 가 0.8 미만이면 결과를 버릴 것.

z 만 볼 거면 tools/check_lidar_z.py 를 쓴다. 벽은 수직면이라 위아래로 미끄러져도
잔차가 안 늘어서, ICP 는 z 에 대해 사실상 눈이 멀어 있다.

같은 검사를 IMU 에 대해서는 중력 방향으로 할 수 있다 (mid360.yaml 의
extrinsic_R 을 적용한 정지 상태 가속도가 base_link 에서 +z 여야 한다).
"""
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

# 드라이버는 xfer_format=0 (PointCloud2) 으로 낸다. frame_id 는 라이다 JSON 의
# 것(livox_top/front/rear)이 찍히고, base_link 로의 변환은 URDF TF 가 준다.
LIDARS = {
    'front': '/livox/lidar_192_168_2_102',
    'rear': '/livox/lidar_192_168_3_144',
    'top': '/livox/lidar_192_168_1_135',
}
TARGET = 'base_link'
FRAMES = 10          # 라이다당 누적 프레임 수
VOXEL = 0.05         # m
MAX_CORR = 0.5       # m, ICP 대응거리


class Collector(Node):
    def __init__(self, topics):
        super().__init__('lidar_overlap_check')
        self.buf = [[] for _ in topics]
        self.frame_id = [None for _ in topics]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        for i, t in enumerate(topics):
            self.create_subscription(
                PointCloud2, t,
                lambda m, i=i: self.cb(m, i),
                qos_profile_sensor_data)

    def cb(self, msg, i):
        if len(self.buf[i]) >= FRAMES:
            return
        self.frame_id[i] = msg.header.frame_id
        # read_points_numpy 는 필드 datatype 이 섞여 있으면 못 쓴다
        # (livox PointCloud2 는 f32/u8/f64 가 섞여 있다).
        a = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.buf[i].append(
            np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64))

    def done(self):
        return all(len(b) >= FRAMES for b in self.buf)

    def lookup(self, frame):
        """base_link <- frame 4x4. 정적 TF 라 시각은 최신으로 본다."""
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


def to_pcd(frames, M):
    pts = np.vstack(frames)
    pts = pts[np.linalg.norm(pts, axis=1) > 0.5]      # blind 영역 제거
    hom = np.hstack([pts, np.ones((len(pts), 1))])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector((M @ hom.T).T[:, :3])
    return pcd.voxel_down_sample(VOXEL)


def main():
    names = sys.argv[1:3] if len(sys.argv) > 2 else ['front', 'rear']
    unknown = [n for n in names if n not in LIDARS]
    if unknown:
        print(f'모르는 라이다: {unknown}. 쓸 수 있는 이름: {list(LIDARS)}')
        return 1
    topics = [LIDARS[n] for n in names]

    rclpy.init()
    node = Collector(topics)
    print(f'{names[0]} / {names[1]} 에서 {FRAMES} 프레임씩 수집 중... '
          f'(차량은 정지해 있어야 한다)')
    t0 = time.time()
    while not node.done() and time.time() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    counts = [len(b) for b in node.buf]

    if not all(counts):
        for t, c in zip(topics, counts):
            print(f'  {t}: {c} 프레임')
        print('한쪽 이상에서 데이터가 안 온다. 드라이버와 IP 를 확인할 것.')
        rclpy.shutdown()
        return 1

    try:
        mats = [node.lookup(f) for f in node.frame_id]
    except tf2_ros.TransformException as e:
        print(f'TF 조회 실패: {e}')
        print('description.launch.py (robot_state_publisher) 가 떠 있는지 확인할 것.')
        return 1
    finally:
        rclpy.shutdown()

    a = to_pcd(node.buf[0], mats[0])
    b = to_pcd(node.buf[1], mats[1])
    print(f'{names[0]} {len(a.points)} 점, {names[1]} {len(b.points)} 점 '
          f'(voxel {VOXEL} m)')

    d = np.asarray(b.compute_point_cloud_distance(a))
    print(f'\n{names[1]} -> {names[0]} 최근접거리  중앙값 {np.median(d):.3f} m, '
          f'90% {np.percentile(d, 90):.3f} m')

    reg = o3d.pipelines.registration.registration_icp(
        b, a, MAX_CORR, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    T = reg.transformation
    dist = np.linalg.norm(T[:3, 3])
    cos = (np.trace(T[:3, :3]) - 1.0) / 2.0
    ang = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    print(f'ICP 잔차 ({names[1]} 를 {names[0]} 에 맞추는 추가 변환):')
    print(f'  이동 {np.round(T[:3, 3], 4).tolist()} m  (크기 {dist:.4f} m)')
    print(f'  회전 {ang:.3f} deg')
    print(f'  fitness {reg.fitness:.3f}, inlier RMSE {reg.inlier_rmse:.4f} m')

    ok = dist < 0.05 and ang < 1.0
    print(f'\n=> {"장착값이 실물과 맞는다" if ok else "장착값이 실물과 어긋난다"}')
    if not ok:
        print(f'   위 잔차만큼 URDF 의 {names[1]}_xyz / {names[1]}_rpy 가 틀렸다는 뜻이다.')
        print('   mount.xacro 를 고치고 check_frames.py 로 나머지 설정을 맞춘 뒤')
        print('   이 검사를 다시 돌릴 것.')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
