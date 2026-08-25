#!/usr/bin/env python3
"""URDF 와 각 설정 파일의 장착값이 서로 맞는지 검사한다.

같은 기하 정보가 세 군데에 있다:
  src/description/urdf/vehicle.urdf.xacro     TF (robot_state_publisher)
  src/livox_merge/config/livox_merge_config.yaml   포인트 병합
  src/fast_lio*/config/mid360.yaml            FAST-LIO 의 lidar->IMU extrinsic

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


def rot(rpy):
    r, p, y = rpy
    def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return Rz(y) @ Ry(p) @ Rx(r)


def urdf_joints(path):
    xml = subprocess.run(['xacro', path], capture_output=True, text=True, check=True).stdout
    out = {}
    for j in ET.fromstring(xml).iter('joint'):
        o = j.find('origin')
        M = np.eye(4)
        M[:3, :3] = rot([float(v) for v in o.get('rpy').split()])
        M[:3, 3] = [float(v) for v in o.get('xyz').split()]
        out[(j.find('parent').get('link'), j.find('child').get('link'))] = M
    return out


def main():
    ok = True
    J = urdf_joints('src/description/urdf/vehicle.urdf.xacro')

    def check(label, a, b, tol=1e-4):
        nonlocal ok
        good = np.allclose(a, b, atol=tol)
        ok &= good
        print(f"  [{'OK' if good else '불일치'}] {label}")
        if not good:
            print(f"        설정: {np.round(np.asarray(a), 5).tolist()}")
            print(f"        URDF: {np.round(np.asarray(b), 5).tolist()}")

    print("livox_merge extrinsics vs URDF")
    cfg = yaml.safe_load(open('src/livox_merge/config/livox_merge_config.yaml'))
    cfg = cfg['merge_lidar_node']['ros__parameters']
    for key, link in [('lidar_0', 'livox_front'), ('lidar_1', 'livox_rear')]:
        M = np.array(cfg[f'lidars.extrinsics.{key}']).reshape(4, 4)
        check(f"{key} <-> base_footprint<-{link}", M, J[('base_footprint', link)])

    # 병합 클라우드가 실제로 놓이는 프레임과 라벨이 같아야 한다
    frame = cfg['output.frame_id']
    print(f"  [{'OK' if frame == 'base_footprint' else '불일치'}] output.frame_id = {frame}")
    ok &= frame == 'base_footprint'

    imu_bf = np.linalg.inv(
        J[('base_footprint', 'livox_top')]
        @ np.block([[np.eye(3), IMU_IN_LIDAR.reshape(3, 1)], [np.zeros((1, 3)), 1.0]]))

    print("\nFAST-LIO extrinsic vs URDF (base_footprint -> IMU)")
    for f in ['src/fast_lio/config/mid360.yaml',
              'src/fast_lio_localization/config/mid360.yaml']:
        txt = open(f).read()
        T = [float(v) for v in re.search(r'extrinsic_T:\s*\[([^\]]+)\]', txt).group(1).split(',')]
        R = re.search(r'extrinsic_R:\s*\[([^\]]+)\]', txt, re.S).group(1)
        R = np.array([float(v) for v in R.replace('\n', ' ').split(',')]).reshape(3, 3)
        check(f"{f} extrinsic_T", T, imu_bf[:3, 3])
        check(f"{f} extrinsic_R", R, imu_bf[:3, :3])

    print("\nURDF 내부 정합")
    check("body<-base_footprint", J[('body', 'base_footprint')], imu_bf)

    f = J[('base_footprint', 'livox_front')][:3, 3]
    r = J[('base_footprint', 'livox_rear')][:3, 3]
    print(f"\n참고: front+rear xy = {np.round((f + r)[:2], 4).tolist()}  (0에 가까울수록 원점대칭)")

    print("\n=> " + ("전부 일치" if ok else "불일치 있음"))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
