#!/usr/bin/env python3
"""조향 드라이버(ID 2/4) 영점 확인 · 현재 위치 영점 재설정.

drive_set.py 에서 "현재 위치를 0 도로" 하는 부분(호밍 방식 35)만 떼어냈다.
drive_set.py 를 그냥 돌리면 방식 25 호밍(리밋 스위치까지 실제로 조향을 돌린다)
과 ±90 도 이동까지 하므로, 영점만 다시 잡고 싶을 때는 이 쪽을 쓴다.

python-can 대신 socketcan(AF_CAN) 을 직접 연다. 이 장비에는 python-can 이
깔려 있지 않아 drive_set.py / homing_set.py 는 그대로는 안 돈다.

  python3 zero_set.py                 # 읽기만: 상태 · 현재각 출력
  python3 zero_set.py --set           # 지금 위치를 0 도로 (바퀴를 정면에 맞춘 뒤)
  python3 zero_set.py --nodes 2       # 특정 노드만
"""

import argparse
import socket
import struct
import sys
import time

# ==========================================
# [설정]
# ==========================================
CHANNEL = 'can0'
NODE_IDS = [2, 4]

COUNTS_PER_REV = 131072
STEER_RATIO = 9.0

MODE_HOMING = 6
HOMING_ZERO_SET_MODE = 35  # 현재 위치를 원점으로 설정

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_NEW_SETPOINT = 0x001F

STATUS_FAULT = 0x0008
STATUS_VOLTAGE_ENABLED = 0x0010
STATUS_TARGET_REACHED = 0x0400
STATUS_HOMING_ATTAINED = 0x1000

# CiA402 상태 머신 (statusword & 0x6F)
STATE_MASK = 0x006F
STATE_OPERATION_ENABLED = 0x0027

NMT_START = 0x01

ZERO_TOLERANCE_PULSE = 200  # 재설정 성공 판정 (약 0.06 도)
# ==========================================

CAN_FRAME = '=IB3x8s'


def open_bus(channel):
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((channel,))
    bus.settimeout(0.05)
    return bus


def drain(bus):
    bus.settimeout(0)
    try:
        while True:
            bus.recv(16)
    except (BlockingIOError, socket.timeout, OSError):
        pass
    bus.settimeout(0.05)


def _send(bus, can_id, payload):
    bus.send(struct.pack(CAN_FRAME, can_id, 8, bytes(payload)))


def _wait_resp(bus, node_id, index, timeout=0.2):
    """0x580+node 에서 같은 index 의 응답을 기다린다. (data[0], data[4:8]) 반환."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            frame = bus.recv(16)
        except (socket.timeout, BlockingIOError):
            continue
        can_id, _dlc, data = struct.unpack(CAN_FRAME, frame)
        if can_id != 0x580 + node_id:
            continue
        if data[1] + (data[2] << 8) != index:
            continue
        return data[0], data[4:8]
    return None, None


def sdo_read(bus, node_id, index, subindex, signed=True):
    drain(bus)
    _send(bus, 0x600 + node_id, [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    cmd, raw = _wait_resp(bus, node_id, index)
    if raw is None or cmd == 0x80:  # 응답 없음 / SDO abort
        return None
    return int.from_bytes(raw, 'little', signed=signed)


def sdo_write(bus, node_id, index, subindex, value, length):
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(length)
    if cmd is None:
        return False
    packed = int(value).to_bytes(length, 'little', signed=(length == 4)) + b'\x00' * (4 - length)
    drain(bus)
    _send(bus, 0x600 + node_id,
          [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex] + list(packed))
    resp_cmd, _ = _wait_resp(bus, node_id, index)
    return resp_cmd == 0x60


def nmt_start(bus, node_id):
    """NMT Start Remote Node. 프리오퍼레이셔널이면 SDO 는 받아도 CiA402 상태
    머신이 안 돈다 - controlword 만 쓰고 끝나서 호밍이 트리거되지 않는다."""
    bus.send(struct.pack(CAN_FRAME, 0x000, 2, bytes([NMT_START, node_id]) + b'\x00' * 6))
    time.sleep(0.2)


def state_name(sw):
    if sw is None:
        return "no-response"
    masked = sw & STATE_MASK
    if masked == STATE_OPERATION_ENABLED:
        return "operation-enabled"
    if (sw & 0x4F) == 0x40:
        return "switch-on-disabled"
    if (sw & 0x4F) == 0x00:
        return "not-ready-to-switch-on"
    if masked == 0x0021:
        return "ready-to-switch-on"
    if masked == 0x0023:
        return "switched-on"
    if (sw & 0x4F) == 0x08:
        return "fault"
    if (sw & 0x4F) == 0x0F:
        return "fault-reaction-active"
    if masked == 0x0007:
        return "quick-stop-active"
    return f"unknown(0x{sw:04X})"


def to_deg(counts):
    return counts / COUNTS_PER_REV / STEER_RATIO * 360.0


def read_state(bus, node_id):
    return {
        'status': sdo_read(bus, node_id, 0x6041, 0x00, signed=False),
        'pos': sdo_read(bus, node_id, 0x6064, 0x00),
        'inputs': sdo_read(bus, node_id, 0x60FD, 0x00, signed=False),
    }


def print_state(bus, nodes, title):
    print(f"\n--- {title} ---")
    for node_id in nodes:
        s = read_state(bus, node_id)
        if s['status'] is None or s['pos'] is None:
            print(f"[Node {node_id}] 응답 없음 (can0 · 노드 ID · 전원 확인)")
            continue
        di = s['inputs'] or 0
        print(
            f"[Node {node_id}] 위치 {s['pos']:>10d} pulse = {to_deg(s['pos']):+7.2f} deg | "
            f"Status 0x{s['status']:04X} {state_name(s['status'])} "
            f"(voltage {'Y' if s['status'] & STATUS_VOLTAGE_ENABLED else 'n'}, "
            f"homing-attained {'Y' if s['status'] & STATUS_HOMING_ATTAINED else 'n'}) | "
            f"NOT {(di >> 0) & 1} POT {(di >> 1) & 1} HOME {(di >> 2) & 1}"
        )


def enable_axis(bus, node_id):
    """NMT Start -> fault reset -> 호밍 모드 -> Operation Enabled 까지. 실패하면 이유를 찍는다."""
    nmt_start(bus, node_id)

    # Fault reset 은 bit7 의 상승 에지가 필요하다 (motor_node 의 init_motor_sdo 와 같은 순서).
    sdo_write(bus, node_id, 0x6040, 0x00, 0x0000, 2)
    time.sleep(0.02)
    sdo_write(bus, node_id, 0x6040, 0x00, 0x0080, 2)
    time.sleep(0.02)

    # 반드시 Operation Enabled 전에 호밍 모드로 바꾼다.
    # Profile Position(1) 상태로 서보를 켜고 0x1F 를 주면 이전 목표위치로 실제 이동한다.
    if not sdo_write(bus, node_id, 0x6060, 0x00, MODE_HOMING, 1):
        print(f"[Node {node_id}] 호밍 모드 쓰기 실패")
        return False
    time.sleep(0.1)
    mode = sdo_read(bus, node_id, 0x6061, 0x00)
    if mode != MODE_HOMING:
        print(f"[Node {node_id}] 호밍 모드 진입 실패 (6061h={mode}, 6=homing 이어야 한다)")
        return False

    if not sdo_write(bus, node_id, 0x6098, 0x00, HOMING_ZERO_SET_MODE, 1):
        print(f"[Node {node_id}] 호밍 방식 35 쓰기 실패")
        return False

    for cw in [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION]:
        if not sdo_write(bus, node_id, 0x6040, 0x00, cw, 2):
            print(f"[Node {node_id}] Control word 0x{cw:04X} 쓰기 실패")
            return False
        time.sleep(0.05)

    # 0x0F 한 번으로 Switched On -> Operation Enabled 로 안 넘어가는 경우가 있다. 재시도.
    status = None
    deadline = time.time() + 1.5
    while time.time() < deadline:
        status = sdo_read(bus, node_id, 0x6041, 0x00, signed=False)
        if status is not None and (status & STATE_MASK) == STATE_OPERATION_ENABLED:
            break
        sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
        time.sleep(0.1)

    if status is None or (status & STATE_MASK) != STATE_OPERATION_ENABLED:
        print(f"[Node {node_id}] Operation Enabled 진입 실패 "
              f"(6041h=0x{status:04X} {state_name(status)})" if status is not None
              else f"[Node {node_id}] 상태 읽기 실패")
        if status is not None and not (status & STATUS_VOLTAGE_ENABLED):
            print(f"[Node {node_id}]   voltage-enabled 비트가 0 이다. 모터 메인 전원 / EMO 확인.")
        if status is not None and (status & STATUS_FAULT):
            code = sdo_read(bus, node_id, 0x603F, 0x00, signed=False)
            print(f"[Node {node_id}]   fault 상태. 603Fh 에러코드 = "
                  f"{'None' if code is None else hex(code)}")
        return False

    return True


def zero_set(bus, nodes):
    """현재 위치를 각 노드의 새로운 0 도로 설정 (호밍 방식 35).

    조향을 물리적으로 움직이지 않는다. 지금 자리를 0 으로 선언할 뿐이다.
    """
    print("\n[System] 현재 위치를 0 도로 재설정 (Homing Mode 35)")

    for node_id in nodes:
        if not enable_axis(bus, node_id):
            return False
        print(f"[Node {node_id}] Operation Enabled · 호밍 모드 35 준비 완료")

    time.sleep(0.05)

    # 0x0F -> 0x1F 상승 에지로 트리거
    for node_id in nodes:
        sdo_write(bus, node_id, 0x6040, 0x00, CW_NEW_SETPOINT, 2)

    # homing-attained 대기
    deadline = time.time() + 3.0
    pending = set(nodes)
    while pending and time.time() < deadline:
        for node_id in list(pending):
            status = sdo_read(bus, node_id, 0x6041, 0x00, signed=False)
            if status is None:
                continue
            if status & STATUS_HOMING_ATTAINED:
                pending.discard(node_id)
        time.sleep(0.05)

    for node_id in nodes:
        sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)

    if pending:
        print(f"[System] homing-attained 비트가 안 올라온 노드: {sorted(pending)}")

    # 실제로 0 이 됐는지 확인. 안 됐으면 성공이라고 말하지 않는다.
    ok = True
    for node_id in nodes:
        pos = sdo_read(bus, node_id, 0x6064, 0x00)
        if pos is None or abs(pos) > ZERO_TOLERANCE_PULSE:
            print(f"[Node {node_id}] 영점이 안 잡혔다 (6064h={pos})")
            ok = False
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--set', action='store_true',
                    help='지금 위치를 0 도로 재설정한다 (기본은 읽기만)')
    ap.add_argument('--nodes', default=','.join(str(n) for n in NODE_IDS),
                    help='대상 노드 ID (기본 2,4)')
    ap.add_argument('--iface', default=CHANNEL, help='CAN 인터페이스 (기본 can0)')
    ap.add_argument('-y', '--yes', action='store_true', help='확인 질문 없이 진행')
    args = ap.parse_args()

    nodes = [int(n) for n in args.nodes.split(',') if n.strip()]

    try:
        bus = open_bus(args.iface)
    except OSError as e:
        print(f"{args.iface} 를 열 수 없다: {e}\n  sudo tools/can_setup.sh up 을 먼저 돌린다.")
        return 1

    try:
        print_state(bus, nodes, "현재 상태")

        if not args.set:
            print("\n영점을 다시 잡으려면 바퀴를 물리적으로 정면에 맞춘 뒤 --set 으로 다시 실행한다.")
            return 0

        if not args.yes:
            print("\n지금 바퀴 위치가 그대로 '0 도' 가 된다. motor_node 는 이 영점을 그대로 믿고")
            print("시동 시 조향을 0 으로 보내므로, 바퀴가 정면이 아닌 상태로 잡으면 안 된다.")
            if input("바퀴가 정면인가? 진행하려면 y: ").strip().lower() != 'y':
                print("취소됨.")
                return 1

        if not zero_set(bus, nodes):
            print("\n영점 재설정 실패.")
            return 1

        time.sleep(0.2)
        print_state(bus, nodes, "재설정 후")
        print("\n영점 재설정 완료. 6064h 가 0 근처면 정상.")
        print("주의: 드라이버 전원을 내리면 다시 풀릴 수 있다 (EEPROM 저장은 하지 않는다).")
        return 0
    except KeyboardInterrupt:
        print("\n중단됨.")
        return 1
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
