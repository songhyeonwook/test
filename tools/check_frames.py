#!/usr/bin/env python3
"""URDF 와 각 설정 파일의 장착값이 서로 맞는지 검사한다.

같은 기하 정보가 세 군데에 있다:
  src/navigation/urdf/mount.xacro            TF 값의 원본 (vehicle urdf 가 include)
  src/livox_merge/config/livox_merge_config.yaml   포인트 병합
  src/bringup/params/livox_merge_bag.yaml          같은 값의 bag 재생용 사본
  src/fast_lio_localization/config/mid360.yaml  병합클라우드->IMU extrinsic,
                                               + tf_2d 의 ref_from_body_*

주의: base_link 가 곧 지면 프레임이다 (2026-08-27 에 base_footprint 를 없애고 합침).
livox_merge extrinsic / FAST-LIO extrinsic_T / tf_2d 의 ref_from_body_* 가 전부
base_link 기준이라, extrinsic_T 와 ref_from_body_xyz 는 같은 값이어야 한다.

한 곳만 고치고 나머지를 안 고치면 조용히 어긋난다. 값을 만질 때마다 돌릴 것.

  python3 tools/check_frames.py
"""
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# MID-360 몸통 원점 -> 내부 IMU (Livox 공장값)
IMU_IN_LIDAR = np.array([0.011, 0.02329, -0.04412])

MERGE_CFGS = [
    'src/livox_merge/config/livox_merge_config.yaml',
    'src/bringup/params/livox_merge_bag.yaml',
]


def rot(rpy):
    r, p, y = rpy
    def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return Rz(y) @ Ry(p) @ Rx(r)


def xlate(t):
    M = np.eye(4)
    M[:3, 3] = t
    return M


def urdf_joints(path):
    """(parent, child) -> 4x4, 그리고 child -> parent 맵."""
    xml = subprocess.run(['xacro', path], capture_output=True, text=True, check=True).stdout
    out, parent_of = {}, {}
    for j in ET.fromstring(xml).iter('joint'):
        o = j.find('origin')
        M = np.eye(4)
        M[:3, :3] = rot([float(v) for v in o.get('rpy').split()])
        M[:3, 3] = [float(v) for v in o.get('xyz').split()]
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        out[(p, c)] = M
        parent_of[c] = p
    return out, parent_of


def make_pose(J, parent_of):
    """base_link 등 임의 조상 기준의 프레임 pose 를 체인으로 합성한다."""
    def pose(frame, root):
        M = np.eye(4)
        cur = frame
        while cur != root:
            if cur not in parent_of:
                raise KeyError(f'{frame} 에서 {root} 로 가는 체인이 끊겼다 ({cur})')
            p = parent_of[cur]
            M = J[(p, cur)] @ M
            cur = p
        return M
    return pose


def main():
    ok = True
    J, parent_of = urdf_joints('src/navigation/urdf/vehicle.urdf.xacro')
    pose = make_pose(J, parent_of)

    def check(label, a, b, tol=1e-4):
        nonlocal ok
        good = np.allclose(a, b, atol=tol)
        ok &= good
        print(f"  [{'OK' if good else '불일치'}] {label}")
        if not good:
            print(f"        설정: {np.round(np.asarray(a), 5).tolist()}")
            print(f"        URDF: {np.round(np.asarray(b), 5).tolist()}")

    # ---- 트리 구조 자체의 약속 ----
    print("TF 트리 구조")
    gone = 'base_footprint' not in parent_of and \
        'base_footprint' not in set(parent_of.values())
    ok &= gone
    print(f"  [{'OK' if gone else '불일치'}] base_footprint 없음 "
          f"(base_link 하나가 지면 프레임)")
    root_ok = parent_of.get('livox_frame') == 'base_link'
    ok &= root_ok
    print(f"  [{'OK' if root_ok else '불일치'}] livox_frame 의 부모 = "
          f"{parent_of.get('livox_frame')} (base_link 이어야 함)")

    # livox_frame 은 front/rear 의 중점이어야 한다
    f_bf = pose('livox_front', 'base_link')[:3, 3]
    r_bf = pose('livox_rear', 'base_link')[:3, 3]
    lf_bf = pose('livox_frame', 'base_link')[:3, 3]
    check("livox_frame == livox_front/livox_rear 의 중점", lf_bf, (f_bf + r_bf) / 2)

    # 세 라이다가 모두 livox_frame 에 매달려 있어야 한다
    for lid in ['livox_top', 'livox_front', 'livox_rear']:
        good = parent_of.get(lid) == 'livox_frame'
        ok &= good
        print(f"  [{'OK' if good else '불일치'}] {lid} 의 부모 = "
              f"{parent_of.get(lid)} (livox_frame 이어야 함)")

    # ---- livox_merge ----
    print("\nlivox_merge extrinsics vs URDF (base_link=지면 기준)")
    for path in MERGE_CFGS:
        cfg = yaml.safe_load(open(path))
        cfg = cfg.get('merge_lidar_node', cfg.get('/**'))['ros__parameters']
        print(f"  {path}")
        for key, link in [('lidar_0', 'livox_front'), ('lidar_1', 'livox_rear')]:
            M = np.array(cfg[f'lidars.extrinsics.{key}']).reshape(4, 4)
            check(f"  {key} <-> base_link<-{link}", M,
                  pose(link, 'base_link'))
        frame = cfg.get('output.frame_id')
        if frame is not None:
            good = frame == 'base_link'
            ok &= good
            print(f"    [{'OK' if good else '불일치'}] output.frame_id = {frame}")

    # ---- FAST-LIO: 병합클라우드(base_link=지면) 를 IMU 에서 본 값 ----
    imu_bf = np.linalg.inv(pose('imu_link', 'base_link'))
    print("\nFAST-LIO extrinsic vs URDF (base_link <- IMU 의 역변환)")
    f = 'src/fast_lio_localization/config/mid360.yaml'
    txt = open(f).read()
    T = [float(v) for v in re.search(r'extrinsic_T:\s*\[([^\]]+)\]', txt).group(1).split(',')]
    R = re.search(r'extrinsic_R:\s*\[([^\]]+)\]', txt, re.S).group(1)
    R = np.array([float(v) for v in R.replace('\n', ' ').split(',')]).reshape(3, 3)
    check(f"{f} extrinsic_T", T, imu_bf[:3, 3])
    check(f"{f} extrinsic_R", R, imu_bf[:3, :3])

    # ---- tf_2d: base_link(=지면) 기준. extrinsic_T 와 같은 값이어야 한다 ----
    imu_bl = np.linalg.inv(pose('imu_link', 'base_link'))
    print("\ntf_2d ref_from_body (body -> base_link) vs URDF")
    loc = yaml.safe_load(open(f))['/**']['ros__parameters']
    M = np.eye(4)
    M[:3, :3] = rot(loc['ref_from_body_rpy'])
    M[:3, 3] = loc['ref_from_body_xyz']
    check("mid360.yaml ref_from_body", M, imu_bl)
    same = np.allclose(T, loc['ref_from_body_xyz'], atol=1e-6)
    ok &= same
    print(f"  [{'OK' if same else '불일치'}] extrinsic_T == ref_from_body_xyz "
          f"(둘 다 base_link<-IMU 라 같아야 한다)")

    print(f"\n참고: front+rear xy(livox_frame 기준) = "
          f"{np.round((pose('livox_front', 'livox_frame')[:3, 3] + pose('livox_rear', 'livox_frame')[:3, 3])[:2], 4).tolist()}"
          f"  (0 이면 원점대칭)")
    print("참고: 지면 위 라이다 높이 = " + ", ".join(
        f"{n} {pose(n, 'base_link')[2, 3]:.4f} m"
        for n in ['livox_front', 'livox_rear', 'livox_top']))

    print("\n=> " + ("전부 일치" if ok else "불일치 있음"))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
