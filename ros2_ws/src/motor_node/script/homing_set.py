#!/usr/bin/env python3

import can
import time
import struct

# ==========================================
# [설정]
# ==========================================
CHANNEL = 'can0'
NODE_IDS = [2, 4]   # <-- 2번과 4번 둘 다 동작
BITRATE = 1000000

# SDO Command Bytes
SDO_READ = 0x40
SDO_WRITE_1BYTE = 0x2F
SDO_WRITE_2BYTE = 0x2B
SDO_WRITE_4BYTE = 0x23
# ==========================================


def send_sdo_write(bus, node_id, index, subindex, data, length):
    """SDO 쓰기 요청을 전송하고 응답을 기다립니다."""
    try:
        if length == 1:
            cmd = SDO_WRITE_1BYTE
        elif length == 2:
            cmd = SDO_WRITE_2BYTE
        elif length == 4:
            cmd = SDO_WRITE_4BYTE
        else:
            return False

        payload = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex]
        data_bytes = list(struct.pack('<I', data))
        payload += data_bytes[:4]

        bus.send(can.Message(
            arbitration_id=0x600 + node_id,
            data=payload,
            is_extended_id=False
        ))

        start = time.time()
        while time.time() - start < 0.2:
            resp = bus.recv(0.01)
            if resp and resp.arbitration_id == 0x580 + node_id and resp.data[0] == 0x60:
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
        bus.send(can.Message(
            arbitration_id=0x600 + node_id,
            data=cmd,
            is_extended_id=False
        ))

        start = time.time()
        while time.time() - start < 0.2:
            resp = bus.recv(0.01)
            if resp and resp.arbitration_id == 0x580 + node_id and len(resp.data) >= 8:
                resp_index = resp.data[1] + (resp.data[2] << 8)
                if resp_index == index:
                    return struct.unpack('<I', resp.data[4:8])[0]
        return None
    except Exception:
        return None


def run_homing_sequence(bus, node_id):
    HOMING_MODE = 25
    HOMING_MODE_DESCRIPTION = "Homing Mode 25"
    HOMING_ZERO_SET_MODE = 35  # 현재 위치를 원점으로 설정

    print(f"\n==============================")
    print(f"Node {node_id}: {HOMING_MODE_DESCRIPTION} 시퀀스 시작")
    print(f"==============================")

    # 1. 호밍 파라미터 설정
    ok = True
    ok &= send_sdo_write(bus, node_id, 0x6060, 0x00, 6, 1)              # Homing mode
    ok &= send_sdo_write(bus, node_id, 0x6098, 0x00, HOMING_MODE, 1)    # Method 25
    ok &= send_sdo_write(bus, node_id, 0x6099, 0x01, 20000, 4)          # Homing speed high
    ok &= send_sdo_write(bus, node_id, 0x6099, 0x02, 5000, 4)           # Homing speed low
    ok &= send_sdo_write(bus, node_id, 0x609A, 0x00, 100000, 4)         # Accel/Decel

    if not ok:
        print(f"[Node {node_id}] 호밍 파라미터 설정 실패")
        return False

    print(f"[Node {node_id}] [1/3] 호밍 모드 {HOMING_MODE} 및 파라미터 설정 완료.")

    # 2. Servo ON (0x06 -> 0x07 -> 0x0F)
    for cmd in [0x06, 0x07, 0x0F]:
        if not send_sdo_write(bus, node_id, 0x6040, 0x00, cmd, 2):
            print(f"[Node {node_id}] Servo ON 실패 (cmd=0x{cmd:02X})")
            return False
        time.sleep(0.1)

    print(f"[Node {node_id}] [2/3] 서보 온(Servo ON) 완료.")

    # 3. 호밍 트리거 (0x0F -> 0x1F)
    if not send_sdo_write(bus, node_id, 0x6040, 0x00, 0x0F, 2):
        print(f"[Node {node_id}] 0x0F 전송 실패")
        return False
    time.sleep(0.1)
    if not send_sdo_write(bus, node_id, 0x6040, 0x00, 0x1F, 2):
        print(f"[Node {node_id}] 0x1F 전송 실패")
        return False

    print(f"[Node {node_id}] [3/3] 호밍 시작 트리거 전송 완료.")

    # 모니터링
    print(f"[Node {node_id}] 호밍 진행 및 NOT/POT/HOME-SWITCH 신호 모니터링 시작")

    HOMING_ATTAINED_BIT = 0x1000  # Status Word Bit 12
    TARGET_REACHED_BIT = 0x0400   # Status Word Bit 10
    HOMING_ERROR_BIT = 0x2000     # Status Word Bit 13
    homing_done = False

    while not homing_done:
        status = read_sdo_robust(bus, node_id, 0x6041, 0x00)
        digital_inputs = read_sdo_robust(bus, node_id, 0x60FD, 0x00)

        if status is not None and digital_inputs is not None:
            not_status = "ON" if (digital_inputs & 0b0001) else "OFF"
            pot_status = "ON" if (digital_inputs & 0b0010) else "OFF"
            home_status = "ON" if (digital_inputs & 0b0100) else "OFF"

            is_fault = status & 0x0008
            is_homing_error = status & HOMING_ERROR_BIT

            if is_fault or is_homing_error:
                print(f"\n[Node {node_id}] 호밍 에러 발생 (Status: 0x{status:04X})")
                return False

            if (status & HOMING_ATTAINED_BIT) and (status & TARGET_REACHED_BIT):
                print(
                    f"\n[Node {node_id}] 호밍 완료! "
                    f"(Status: 0x{status:04X}, NOT: {not_status}, POT: {pot_status}, HOME: {home_status})"
                )
                homing_done = True
                send_sdo_write(bus, node_id, 0x6040, 0x00, 0x0F, 2)
                break

            print(
                f"[Node {node_id}] 진행 중... "
                f"Status: 0x{status:04X}, Digital Inputs: 0x{digital_inputs:04X}, "
                f"NOT: {not_status}, POT: {pot_status}, HOME: {home_status}",
                end="\r"
            )

        time.sleep(0.1)

    # 호밍 완료 후 현재 위치를 0으로 초기화
    if homing_done:
        print(f"\n[Node {node_id}] 현재 위치 0 초기화 시작 (Homing Mode 35 사용)")

        if not send_sdo_write(bus, node_id, 0x6098, 0x00, HOMING_ZERO_SET_MODE, 1):
            print(f"[Node {node_id}] Mode 35 설정 실패")
            return False

        send_sdo_write(bus, node_id, 0x6040, 0x00, 0x0F, 2)
        time.sleep(0.05)

        if send_sdo_write(bus, node_id, 0x6040, 0x00, 0x1F, 2):
            print(f"[Node {node_id}] Control Word 0x1F 전송 완료. 모드 {HOMING_ZERO_SET_MODE} 트리거됨.")
            time.sleep(0.1)

            current_pos_raw = read_sdo_robust(bus, node_id, 0x6064, 0x00)

            if current_pos_raw is not None:
                actual_position = struct.unpack(
                    '<i',
                    current_pos_raw.to_bytes(4, byteorder='little')
                )[0]
                print(f"[Node {node_id}] 위치 초기화 후 6064h 현재 값: {actual_position}")

            send_sdo_write(bus, node_id, 0x6040, 0x00, 0x0F, 2)
            return True
        else:
            print(f"[Node {node_id}] 위치 0 초기화 트리거(0x1F) 실패.")
            return False

    return False


def main():
    try:
        bus = can.Bus(channel=CHANNEL, interface='socketcan', bitrate=BITRATE)

        results = {}
        for node_id in NODE_IDS:
            result = run_homing_sequence(bus, node_id)
            results[node_id] = result
            time.sleep(0.3)  # 노드 간 약간의 여유

        print("\n========== 최종 결과 ==========")
        for node_id, result in results.items():
            print(f"Node {node_id}: {'성공' if result else '실패'}")

    except KeyboardInterrupt:
        print("\n중단됨.")
    except can.CanError as e:
        print(f"\nCAN 통신 오류가 발생했습니다: {e}")
    except Exception as e:
        print(f"\n예기치 않은 오류가 발생했습니다: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()


if __name__ == "__main__":
    main()
