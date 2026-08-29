#!/usr/bin/env python3

import can
import time
import struct

# ==========================================
# [설정]
# ==========================================
CHANNEL = 'can0'
NODE_IDS = [2, 4]
BITRATE = 1000000

# SDO Command Bytes
SDO_READ = 0x40
SDO_WRITE_1BYTE = 0x2F
SDO_WRITE_2BYTE = 0x2B
SDO_WRITE_4BYTE = 0x23

# 분해능 설정
COUNTS_PER_REV = 131072
STEER_RATIO = 10.0  # motor_node.cpp 의 STEER_RATIO 와 같아야 한다

# 목표 펄스 계산
DEG_90 = int(90 * (STEER_RATIO * COUNTS_PER_REV / 360.0))
DEG_MINUS_90 = int(-90 * (STEER_RATIO * COUNTS_PER_REV / 360.0))

# CiA402 / Mode
MODE_PROFILE_POSITION = 1
MODE_HOMING = 6

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_NEW_SETPOINT = 0x001F

STATUS_TARGET_REACHED = 0x0400
STATUS_HOMING_ATTAINED = 0x1000
STATUS_HOMING_ERROR = 0x2000
STATUS_FAULT = 0x0008
# ==========================================


def send_sdo_write(bus, node_id, index, subindex, data, length):
    """SDO 쓰기 요청을 전송하고 응답을 기다립니다."""
    try:
        if length == 1:
            cmd = SDO_WRITE_1BYTE
            packed = struct.pack('<B', data & 0xFF) + b'\x00\x00\x00'
        elif length == 2:
            cmd = SDO_WRITE_2BYTE
            packed = struct.pack('<H', data & 0xFFFF) + b'\x00\x00'
        elif length == 4:
            cmd = SDO_WRITE_4BYTE
            packed = struct.pack('<i', int(data))
        else:
            return False

        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        payload += list(packed)

        bus.send(
            can.Message(
                arbitration_id=0x600 + node_id,
                data=payload,
                is_extended_id=False
            )
        )

        start = time.time()
        while time.time() - start < 0.2:
            resp = bus.recv(0.01)
            if resp and resp.arbitration_id == 0x580 + node_id and len(resp.data) >= 4:
                if resp.data[0] == 0x60:
                    return True
        return False
    except Exception:
        return False


def read_sdo_robust(bus, node_id, index, subindex):
    """SDO 읽기 요청을 전송하고 응답을 받아 데이터를 반환합니다."""
    try:
        while bus.recv(0):
            pass

        cmd = [SDO_READ, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        bus.send(
            can.Message(
                arbitration_id=0x600 + node_id,
                data=cmd,
                is_extended_id=False
            )
        )

        start = time.time()
        while time.time() - start < 0.2:
            resp = bus.recv(0.01)
            if resp and resp.arbitration_id == 0x580 + node_id and len(resp.data) >= 8:
                resp_index = resp.data[1] + (resp.data[2] << 8)
                if resp_index == index:
                    cmd_byte = resp.data[0]

                    # expedited upload 응답 처리
                    if cmd_byte in [0x4F, 0x4B, 0x47, 0x43]:
                        data_bytes = bytes(resp.data[4:8])

                        if cmd_byte == 0x4F:   # 1 byte
                            return struct.unpack('<b', data_bytes[:1])[0]
                        elif cmd_byte == 0x4B: # 2 bytes
                            return struct.unpack('<h', data_bytes[:2])[0]
                        else:                  # 4 bytes
                            return struct.unpack('<i', data_bytes)[0]
        return None
    except Exception:
        return None


def send_nmt_start(bus, node_id):
    """CANopen NMT Start Remote Node"""
    try:
        bus.send(
            can.Message(
                arbitration_id=0x000,
                data=[0x01, node_id],
                is_extended_id=False
            )
        )
        time.sleep(0.1)
        return True
    except Exception:
        return False


def activate_mode(bus, node_id, mode):
    """
    motor_node.cpp의 init_motor_sdo() 스타일 반영
    - NMT Start
    - 0x6060 mode 설정
    - mode별 파라미터 설정
    - 0x6040: 0x06 -> 0x07 -> 0x0F
    """
    if not send_nmt_start(bus, node_id):
        print(f"[Node {node_id}] NMT Start 실패")
        return False

    if not send_sdo_write(bus, node_id, 0x6060, 0x00, mode, 1):
        print(f"[Node {node_id}] 6060h 모드 설정 실패 (mode={mode})")
        return False

    # mode display 확인
    time.sleep(0.05)
    mode_display = read_sdo_robust(bus, node_id, 0x6061, 0x00)
    if mode_display is not None:
        print(f"[Node {node_id}] Mode display(6061h): {mode_display}")

    # 모드별 기본 파라미터
    if mode == MODE_PROFILE_POSITION:
        ok = True
        ok &= send_sdo_write(bus, node_id, 0x6081, 0x00, 50000, 4)   # Profile velocity
        ok &= send_sdo_write(bus, node_id, 0x6083, 0x00, 50000, 4)   # Acceleration
        ok &= send_sdo_write(bus, node_id, 0x6084, 0x00, 50000, 4)   # Deceleration
        if not ok:
            print(f"[Node {node_id}] Position mode 파라미터 설정 실패")
            return False

    # Servo ON / Operation Enable
    for cw in [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION]:
        if not send_sdo_write(bus, node_id, 0x6040, 0x00, cw, 2):
            print(f"[Node {node_id}] Control Word 전송 실패: 0x{cw:04X}")
            return False
        time.sleep(0.05)

    status = read_sdo_robust(bus, node_id, 0x6041, 0x00)
    if status is not None:
        print(f"[Node {node_id}] Status after mode activate: 0x{status:04X}")

    return True


def is_homing_already_done(bus, node_id):
    status = read_sdo_robust(bus, node_id, 0x6041, 0x00)
    if status is not None:
        return (status & STATUS_HOMING_ATTAINED) != 0
    return False


def run_homing_simultaneous(bus, nodes):
    HOMING_MODE = 25
    HOMING_ZERO_SET_MODE = 35

    print("\n========================================")
    print(f"Nodes {nodes}: 동시 Homing Mode 25 시퀀스 시작")
    print("========================================")

    # 1. Homing mode 활성화 + 파라미터 설정
    for node_id in nodes:
        if not activate_mode(bus, node_id, MODE_HOMING):
            print(f"[Node {node_id}] Homing mode 활성화 실패")
            return False

        ok = True
        ok &= send_sdo_write(bus, node_id, 0x6098, 0x00, HOMING_MODE, 1)
        ok &= send_sdo_write(bus, node_id, 0x6099, 0x01, 20000, 4)
        ok &= send_sdo_write(bus, node_id, 0x6099, 0x02, 5000, 4)
        ok &= send_sdo_write(bus, node_id, 0x609A, 0x00, 100000, 4)

        if not ok:
            print(f"[Node {node_id}] 호밍 파라미터 설정 실패")
            return False

    print("[System] [1/3] Homing mode 활성화 및 파라미터 설정 완료.")

    # 2. Homing trigger 준비
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
    time.sleep(0.1)

    # 3. Homing trigger (0x1F)
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_NEW_SETPOINT, 2)

    print("[System] [2/3] 호밍 시작 트리거 전송 완료. 모니터링 시작...")

    done_nodes = set()

    while len(done_nodes) < len(nodes):
        for node_id in nodes:
            if node_id in done_nodes:
                continue

            status = read_sdo_robust(bus, node_id, 0x6041, 0x00)
            digital_inputs = read_sdo_robust(bus, node_id, 0x60FD, 0x00)

            if status is not None and digital_inputs is not None:
                home_status = "ON" if (digital_inputs & 0b0100) else "OFF"
                print(
                    f"[Node {node_id}] 진행 중... "
                    f"Status: 0x{status:04X}, DI: 0x{digital_inputs & 0xFFFFFFFF:08X}, HOME: {home_status}    ",
                    end="\r"
                )

                if (status & STATUS_FAULT) or (status & STATUS_HOMING_ERROR):
                    print(f"\n[Node {node_id}] 호밍 에러 발생 (Status: 0x{status:04X})")
                    return False

                if (status & STATUS_HOMING_ATTAINED) and (status & STATUS_TARGET_REACHED):
                    print(f"\n[Node {node_id}] 호밍 완료! (Status: 0x{status:04X}, HOME: {home_status})")
                    send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
                    done_nodes.add(node_id)

        time.sleep(0.1)

    print("\n[System] [3/3] 현재 위치 0 초기화 시작 (Homing Mode 35 사용)")

    for node_id in nodes:
        if not activate_mode(bus, node_id, MODE_HOMING):
            print(f"[Node {node_id}] Mode 35 진입 전 Homing mode 활성화 실패")
            return False

        send_sdo_write(bus, node_id, 0x6098, 0x00, HOMING_ZERO_SET_MODE, 1)
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)

    time.sleep(0.05)

    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_NEW_SETPOINT, 2)

    time.sleep(0.1)

    for node_id in nodes:
        current_pos_raw = read_sdo_robust(bus, node_id, 0x6064, 0x00)
        print(f"[Node {node_id}] 위치 초기화 후 6064h 값: {current_pos_raw}")
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)

    return True


def move_targets_simultaneous(bus, target_dict):
    nodes = list(target_dict.keys())

    print("\n========================================")
    print("동시 위치 이동 시작")
    for n, t in target_dict.items():
        print(f" - Node {n} -> 목표: {t} pulse")
    print("========================================")

    # 1. Position mode 활성화 + 이동 파라미터 설정
    for node_id, target_pulse in target_dict.items():
        if not activate_mode(bus, node_id, MODE_PROFILE_POSITION):
            print(f"[Node {node_id}] Position mode 활성화 실패")
            return False

        ok = True
        ok &= send_sdo_write(bus, node_id, 0x6081, 0x00, 50000, 4)
        ok &= send_sdo_write(bus, node_id, 0x6083, 0x00, 100000, 4)
        ok &= send_sdo_write(bus, node_id, 0x6084, 0x00, 100000, 4)
        ok &= send_sdo_write(bus, node_id, 0x607A, 0x00, target_pulse, 4)

        if not ok:
            print(f"[Node {node_id}] 목표 위치 파라미터 설정 실패")
            return False

    # 2. 이동 트리거 준비
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
    time.sleep(0.1)

    # 3. 이동 트리거
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_NEW_SETPOINT, 2)

    print("[System] 위치 이동 트리거 전송 완료. 도달 대기 중...")

    # 4. 모니터링
    done_nodes = set()
    while len(done_nodes) < len(nodes):
        for node_id in nodes:
            if node_id in done_nodes:
                continue

            status = read_sdo_robust(bus, node_id, 0x6041, 0x00)
            if status is not None:
                print(f"[Node {node_id}] 이동 중... Status: 0x{status:04X}     ", end="\r")

                if status & STATUS_TARGET_REACHED:
                    print(f"\n[Node {node_id}] 목표 위치 도착 완료. (Status: 0x{status:04X})")
                    send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
                    done_nodes.add(node_id)

        time.sleep(0.1)

    return True


def zero_current_position_simultaneous(bus, nodes):
    """현재 위치를 각 노드의 새로운 0도로 설정"""
    HOMING_ZERO_SET_MODE = 35

    print("\n========================================")
    print("현재 위치를 0도로 재설정 시작")
    print("========================================")

    # 1) Homing mode 활성화 + Mode 35 설정
    for node_id in nodes:
        if not activate_mode(bus, node_id, MODE_HOMING):
            print(f"[Node {node_id}] 현재 위치 0도 재설정용 Homing mode 활성화 실패")
            return False
        send_sdo_write(bus, node_id, 0x6098, 0x00, HOMING_ZERO_SET_MODE, 1)

    time.sleep(0.05)

    # 2) 트리거 준비
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)
    time.sleep(0.05)

    # 3) Mode 35 트리거
    for node_id in nodes:
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_NEW_SETPOINT, 2)
    time.sleep(0.1)

    # 4) 결과 확인
    for node_id in nodes:
        current_pos_raw = read_sdo_robust(bus, node_id, 0x6064, 0x00)
        print(f"[Node {node_id}] 0도 재설정 후 6064h 값: {current_pos_raw}")
        send_sdo_write(bus, node_id, 0x6040, 0x00, CW_ENABLE_OPERATION, 2)

    print("[System] 현재 위치 0도 재설정 완료")
    return True


def main():
    try:
        bus = can.Bus(channel=CHANNEL, interface='socketcan', bitrate=BITRATE)

        # 호밍 단계
        nodes_to_home = [n for n in NODE_IDS if not is_homing_already_done(bus, n)]

        if len(nodes_to_home) == 0:
            print("\n[System] 지정된 모든 노드가 이미 호밍되어 있습니다.")
        else:
            if not run_homing_simultaneous(bus, nodes_to_home):
                print("\n[System] 호밍 실패. 프로그램을 중단합니다.")
                return
            time.sleep(0.5)

        # 위치 이동 단계
        targets = {2: DEG_90, 4: DEG_MINUS_90}
        if not move_targets_simultaneous(bus, targets):
            print("\n[System] 위치 이동 실패. 프로그램을 중단합니다.")
            return

        # 이동 완료 후 현재 위치를 다시 0도로 설정
        time.sleep(0.3)
        if not zero_current_position_simultaneous(bus, list(targets.keys())):
            print("\n[System] 현재 위치 0도 재설정 실패. 프로그램을 중단합니다.")
            return

        print("\n" + "=" * 30)
        print("drive ready")
        print("=" * 30)

    except KeyboardInterrupt:
        print("\n사용자에 의해 강제 중단됨.")
    except Exception as e:
        print(f"\n예기치 않은 오류 발생: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()


if __name__ == "__main__":
    main()
